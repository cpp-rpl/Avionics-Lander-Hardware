/**
 * app_rtos.h — RPL flight computer FreeRTOS demo (STM32H723ZG / NUCLEO-H723ZG)
 *
 * Drop-in application layer for a CubeMX-generated project with FreeRTOS
 * (CMSIS_V2 middleware). Uses the native FreeRTOS API underneath.
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "main.h"

/* Call once, before the scheduler starts (freertos.c -> USER CODE BEGIN RTOS_THREADS). */
void App_Init(void);

/* Call from HAL_GPIO_EXTI_Callback() when the user button (PC13) fires. */
void App_ButtonISR(void);

/* Call from the CubeMX-generated hook stubs in freertos.c. */
void App_OnStackOverflow(const char *taskName);
void App_OnMallocFailed(void);

#endif /* APP_RTOS_H */
