/**
 * app_rtos.c — RPL flight computer FreeRTOS demo (STM32H723ZG / NUCLEO-H723ZG)
 *
 * Task layout (mirrors the real flight software architecture):
 *
 *   Task       Prio  Rate        Role today                 Becomes on RPL
 *   ---------  ----  ----------  -------------------------  ---------------------------
 *   Abort      48    event       button ISR -> semaphore    abort / kill-switch handler
 *   Sensor     40    100 Hz      simulated descent sample   IMU / baro / altimeter drivers
 *   Telemetry  32    queue-fed   CSV over UART (10 Hz)      radio downlink / SD logging
 *   Monitor    16    0.5 Hz      heap/stack/queue health    health & fault monitoring
 *   Heartbeat  16    2 Hz        green LED blink            "alive" indicator
 *
 * Concepts exercised: periodic tasks (vTaskDelayUntil), priorities/preemption,
 * queues (producer/consumer), mutex (shared UART), ISR->task sync (binary
 * semaphore + FromISR API), stack/heap instrumentation, overrun detection,
 * fault hooks.
 */
#include "app_rtos.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Board mapping — NUCLEO-H723ZG                                              */
/* ------------------------------------------------------------------------- */
extern UART_HandleTypeDef huart3;           /* ST-LINK virtual COM port (PD8/PD9) */
#define APP_UART            (&huart3)

#define LED_GREEN_PORT      GPIOB           /* LD1 */
#define LED_GREEN_PIN       GPIO_PIN_0
#define LED_YELLOW_PORT     GPIOE           /* LD2 */
#define LED_YELLOW_PIN      GPIO_PIN_1
#define LED_RED_PORT        GPIOB           /* LD3 */
#define LED_RED_PIN         GPIO_PIN_14

/* ------------------------------------------------------------------------- */
/* Tunables                                                                   */
/* ------------------------------------------------------------------------- */
/* CubeMX CMSIS_V2 uses configMAX_PRIORITIES = 56 (0..55) and the CMSIS
 * names map onto it: Idle=1, Low=8, BelowNormal=16, Normal=24 (CubeMX's
 * defaultTask), AboveNormal=32, High=40, Realtime=48. Our critical tasks
 * must sit ABOVE Normal or defaultTask would outrank them. */
#define PRIO_ABORT          48      /* osPriorityRealtime    */
#define PRIO_SENSOR         40      /* osPriorityHigh        */
#define PRIO_TELEM          32      /* osPriorityAboveNormal */
#define PRIO_MONITOR        16      /* osPriorityBelowNormal */
#define PRIO_HEARTBEAT      16      /* osPriorityBelowNormal */

/* Stack sizes are in WORDS (4 bytes each on Cortex-M7). Tasks that call
 * vsnprintf get more headroom. Monitor output shows the real margin. */
#define STACK_ABORT         384
#define STACK_SENSOR        256
#define STACK_TELEM         512
#define STACK_MONITOR       512
#define STACK_HEARTBEAT     128

#define SENSOR_PERIOD_MS    10              /* 100 Hz */
#define TELEM_DECIMATE      10              /* print every 10th sample -> 10 Hz */
#define MONITOR_PERIOD_MS   2000
#define HEARTBEAT_MS        500
#define SAMPLE_QUEUE_LEN    16
#define BUTTON_DEBOUNCE_MS  200

/* ------------------------------------------------------------------------- */
/* Types & shared state                                                       */
/* ------------------------------------------------------------------------- */
typedef struct {
    uint32_t seq;
    uint32_t tick;          /* RTOS tick when sampled (1 tick = 1 ms) */
    int32_t  alt_mm;        /* altitude above ground */
    int32_t  vz_mms;        /* vertical velocity, negative = descending */
    int16_t  acc_mg[3];     /* accelerometer, milli-g */
} sample_t;

static QueueHandle_t     q_samples;
static SemaphoreHandle_t m_uart;            /* mutex: one writer on the UART at a time */
static SemaphoreHandle_t s_abort;           /* binary semaphore: ISR -> Abort task */

static TaskHandle_t h_abort, h_sensor, h_telem, h_monitor, h_heartbeat;

static volatile uint32_t g_dropped;         /* samples lost because queue was full */
static volatile uint32_t g_overruns;        /* sensor loop missed its 10 ms deadline */
static volatile uint32_t g_abort_events;
static volatile TickType_t g_isr_tick;      /* tick captured in the ISR (latency calc) */

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */

/* Thread-safe logger. Only call from task context (never from an ISR). */
static void log_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof buf - 1) n = (int)sizeof buf - 1;

    if (xSemaphoreTake(m_uart, pdMS_TO_TICKS(50)) == pdTRUE) {
        HAL_UART_Transmit(APP_UART, (uint8_t *)buf, (uint16_t)n, 100);
        xSemaphoreGive(m_uart);
    }
}

/* Tiny LCG for fake sensor noise — deterministic, no libc rand(). */
static uint32_t rng_state = 0x1234ABCDu;
static int16_t noise(int16_t span)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (int16_t)((int32_t)(rng_state >> 16) % (2 * span + 1) - span);
}

/* ------------------------------------------------------------------------- */
/* Tasks                                                                      */
/* ------------------------------------------------------------------------- */

/* 100 Hz producer. Simulates a powered descent: fast descent to 2 m,
 * slow final descent to touchdown, sit on the pad, then replay. */
static void SensorTask(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    sample_t s = {0};
    int32_t alt_mm = 10000;
    int32_t vz_mms = 0;
    uint32_t landed_samples = 0;

    for (;;) {
        /* --- "read sensors" (simulated) --- */
        if (alt_mm > 2000)      vz_mms = -1500;
        else if (alt_mm > 0)    vz_mms = -300;
        else                  { vz_mms = 0; alt_mm = 0; }

        alt_mm += vz_mms * (int32_t)SENSOR_PERIOD_MS / 1000;
        if (alt_mm < 0) alt_mm = 0;

        if (alt_mm == 0 && ++landed_samples > 300) {   /* 3 s on the pad, then replay */
            alt_mm = 10000;
            landed_samples = 0;
        }

        s.seq++;
        s.tick      = (uint32_t)xTaskGetTickCount();
        s.alt_mm    = alt_mm;
        s.vz_mms    = vz_mms;
        s.acc_mg[0] = noise(15);
        s.acc_mg[1] = noise(15);
        s.acc_mg[2] = (int16_t)(1000 + noise(25));

        /* --- publish; never block the control-rate task --- */
        if (xQueueSend(q_samples, &s, 0) != pdPASS) {
            g_dropped++;
        }

        /* --- deadline check: did this iteration eat the whole period? --- */
        if ((xTaskGetTickCount() - last_wake) >= period) {
            g_overruns++;
        }

        vTaskDelayUntil(&last_wake, period);    /* fixed-rate, no drift */
    }
}

/* Consumer. Blocks on the queue, emits CSV at 10 Hz. */
static void TelemetryTask(void *arg)
{
    (void)arg;
    sample_t s;

    log_printf("# CSV: T,seq,tick_ms,alt_mm,vz_mms,ax_mg,ay_mg,az_mg\r\n");

    for (;;) {
        if (xQueueReceive(q_samples, &s, portMAX_DELAY) == pdTRUE) {
            if ((s.seq % TELEM_DECIMATE) == 0) {
                log_printf("T,%lu,%lu,%ld,%ld,%d,%d,%d\r\n",
                           (unsigned long)s.seq, (unsigned long)s.tick,
                           (long)s.alt_mm, (long)s.vz_mms,
                           s.acc_mg[0], s.acc_mg[1], s.acc_mg[2]);
            }
        }
    }
}

/* Highest priority. Sleeps until the button ISR gives the semaphore. */
static void AbortTask(void *arg)
{
    (void)arg;
    TickType_t last_event = 0;

    for (;;) {
        xSemaphoreTake(s_abort, portMAX_DELAY);

        TickType_t now = xTaskGetTickCount();
        if (g_abort_events > 0 && (now - last_event) < pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
            continue;                           /* switch bounce */
        }
        last_event = now;
        g_abort_events++;

        HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
        log_printf("# EVENT abort #%lu  isr_tick=%lu  task_tick=%lu  latency=%lu ms\r\n",
                   (unsigned long)g_abort_events, (unsigned long)g_isr_tick,
                   (unsigned long)now, (unsigned long)(now - g_isr_tick));
    }
}

/* System health: heap, per-task stack margin, queue depth, error counters. */
static void MonitorTask(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    log_printf("\r\n# ==== RPL flight computer — FreeRTOS demo ====\r\n");
    log_printf("# MCU STM32H723ZG  SYSCLK=%lu Hz  tick=%u Hz  heap_free=%u B\r\n",
               (unsigned long)HAL_RCC_GetSysClockFreq(),
               (unsigned)configTICK_RATE_HZ,
               (unsigned)xPortGetFreeHeapSize());

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MONITOR_PERIOD_MS));
        HAL_GPIO_TogglePin(LED_YELLOW_PORT, LED_YELLOW_PIN);

        log_printf("# MON up=%lus heap=%u min_heap=%u q=%lu/%u drop=%lu ovr=%lu abort=%lu\r\n",
                   (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ),
                   (unsigned)xPortGetFreeHeapSize(),
                   (unsigned)xPortGetMinimumEverFreeHeapSize(),
                   (unsigned long)uxQueueMessagesWaiting(q_samples),
                   (unsigned)SAMPLE_QUEUE_LEN,
                   (unsigned long)g_dropped,
                   (unsigned long)g_overruns,
                   (unsigned long)g_abort_events);

        /* High-water mark = least free stack ever seen, in words. Near 0 = danger. */
        log_printf("# STK(words free) abort=%lu sensor=%lu telem=%lu mon=%lu hb=%lu\r\n",
                   (unsigned long)uxTaskGetStackHighWaterMark(h_abort),
                   (unsigned long)uxTaskGetStackHighWaterMark(h_sensor),
                   (unsigned long)uxTaskGetStackHighWaterMark(h_telem),
                   (unsigned long)uxTaskGetStackHighWaterMark(h_monitor),
                   (unsigned long)uxTaskGetStackHighWaterMark(h_heartbeat));
    }
}

static void HeartbeatTask(void *arg)
{
    (void)arg;
    for (;;) {
        HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN);
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_MS));
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */
void App_Init(void)
{
    q_samples = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(sample_t));
    m_uart    = xSemaphoreCreateMutex();
    s_abort   = xSemaphoreCreateBinary();
    configASSERT(q_samples && m_uart && s_abort);

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(AbortTask,     "Abort",     STACK_ABORT,     NULL, PRIO_ABORT,     &h_abort);
    ok &= xTaskCreate(SensorTask,    "Sensor",    STACK_SENSOR,    NULL, PRIO_SENSOR,    &h_sensor);
    ok &= xTaskCreate(TelemetryTask, "Telemetry", STACK_TELEM,     NULL, PRIO_TELEM,     &h_telem);
    ok &= xTaskCreate(MonitorTask,   "Monitor",   STACK_MONITOR,   NULL, PRIO_MONITOR,   &h_monitor);
    ok &= xTaskCreate(HeartbeatTask, "Heartbeat", STACK_HEARTBEAT, NULL, PRIO_HEARTBEAT, &h_heartbeat);
    configASSERT(ok == pdPASS);
}

void App_ButtonISR(void)
{
    if (s_abort == NULL) return;                /* scheduler objects not ready yet */

    BaseType_t higher_prio_woken = pdFALSE;
    g_isr_tick = xTaskGetTickCountFromISR();
    xSemaphoreGiveFromISR(s_abort, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);      /* switch straight to Abort task on exit */
}

void App_OnStackOverflow(const char *taskName)
{
    (void)taskName;                             /* inspect in debugger */
    taskDISABLE_INTERRUPTS();
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
    for (;;) { }                                /* solid red = stack overflow */
}

void App_OnMallocFailed(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {                                  /* fast red blink = heap exhausted */
        HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
        for (volatile uint32_t i = 0; i < 2000000u; i++) { }
    }
}
