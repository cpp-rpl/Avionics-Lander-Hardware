# FreeRTOS Proof of Concept Guide

## Setup

### 1. Install the tools

| Tool | What it's for | macOS install |
| --- | --- | --- |
| Arm GNU Toolchain | Compiler for the STM32 | `brew install --cask gcc-arm-embedded` |
| CMake + Ninja | Build system | `brew install cmake ninja` |
| STM32CubeProgrammer | Flashing the board | Download from st.com (free account) |
| STM32CubeMX | Changing pins/clocks/FreeRTOS settings | Download from st.com (only if you change hardware config) |
| Python packages | Capturing and plotting serial output | `pip3 install pyserial matplotlib` |

Check it worked: `arm-none-eabi-gcc --version` and `cmake --version` both print a version.

### 2. Get the project

Clone this repo and use the `firmware/freertos-poc` folder (the CubeMX project inside is named `rpl_tst1`). Use a path with no spaces in it, since spaces can break the CMake build. Then go to **Build and flash**.

### 3. How the project was created

This is already done in the project folder. Follow it only to rebuild the project from scratch or to understand the settings.

1. **New project:** STM32CubeMX → New Project → Board Selector → `NUCLEO-H723ZG`. Answer Yes to default peripherals, then click **Unselect All** on the board components popup (we don't use ST's BSP).
2. **Pins:** set PB0 / PE1 / PB14 to `GPIO_Output` (labels `LED_GREEN` / `LED_YELLOW` / `LED_RED`). Set PC13 to `GPIO_EXTI13` (label `USER_BTN`, rising edge, no pull). Set USART3 to Asynchronous, 115200 baud, on PD8/PD9.
3. **Timebase:** System Core → SYS → Timebase Source → `TIM6`. FreeRTOS needs SysTick for itself.
4. **FreeRTOS:** Middleware → FREERTOS → Interface `CMSIS_V2`.
   - Config parameters: `TOTAL_HEAP_SIZE` = 32768, memory scheme `heap_4`, `USE_MUTEXES` on, `CHECK_FOR_STACK_OVERFLOW` = Option2, `USE_MALLOC_FAILED_HOOK` on.
   - Include parameters: `vTaskDelayUntil` and `uxTaskGetStackHighWaterMark` on.
   - Advanced settings: `USE_NEWLIB_REENTRANT` on.
5. **Button interrupt:** System Core → NVIC → enable `EXTI line[15:10]`, priority 5. Any interrupt that talks to FreeRTOS must be priority 5 or higher in number.
6. **Generate:** Project Manager → Toolchain/IDE = `CMake` → Generate Code. Say Yes if it asks to download the STM32H7 firmware package.
7. **Add the code:**
   - Copy `app_rtos.c` to `Core/Src/` and `app_rtos.h` to `Core/Inc/`.
   - Add `Core/Src/app_rtos.c` under "Add user sources here" in the top-level `CMakeLists.txt`.
   - Make the `main.c` and `freertos.c` edits listed under **The code** below.

When you regenerate from CubeMX, it keeps anything between `/* USER CODE BEGIN */` and `/* USER CODE END */` and overwrites everything else. Keep all edits to generated files inside those blocks.

## The code

| File | What it does |
| --- | --- |
| `Core/Src/app_rtos.c` | All the code: the 5 tasks, the queue, mutex and semaphore, and the fault handlers |
| `Core/Inc/app_rtos.h` | Declares the 4 functions the generated code calls: `App_Init`, `App_ButtonISR`, `App_OnStackOverflow`, `App_OnMallocFailed` |
| `Core/Src/main.c` | CubeMX-generated startup.  Edits: `App_Init()` in the `RTOS_THREADS` block, and `HAL_GPIO_EXTI_Callback()` in block 4 (calls `App_ButtonISR()` when PC13 fires) |
| `Core/Src/freertos.c` | CubeMX-generated FreeRTOS hooks. Edits:  calls to `App_OnStackOverflow()` / `App_OnMallocFailed()` |
| `rpl_tst1.ioc` | The CubeMX config file. Open it in CubeMX to change pins or settings |
| `CMakeLists.txt` | Build file. The only edit is listing `app_rtos.c` |
| `rpl_log.py` | Captures serial output to a log file and plots it |
| `Drivers/`, `Middlewares/`, `cmake/` | ST's HAL, the FreeRTOS kernel and build config. Generated; don't edit |

## How it works

```
 Blue button ──press──> Button ISR ──give──> Semaphore ──wakes──> Abort task (prio 48) ──┐
 (PC13)                 (EXTI, NVIC 5)       (s_abort)            red LED + log           │
                                                                                          ├──> UART mutex ──> USART3 ──> USB serial
 RTOS tick ──wakes────> Sensor task ───────> Queue ─────────────> Telemetry (prio 32) ────┤    (m_uart)        115200 baud
 (SysTick, 1 ms)        (prio 40, 10 ms)     (q_samples, 16)      CSV at 10 Hz            │
                                                                  Monitor (prio 16) ──────┘
                                                                  every 2 s
```

Read it left to right: the 1 ms tick wakes Sensor, samples pass through the queue to Telemetry, and the button ISR wakes Abort through the semaphore. Every task that prints goes through the UART mutex. Heartbeat (green LED) runs on its own and isn't shown.

| Task | Priority | What it does |
| --- | --- | --- |
| Abort | 48 (highest) | Wakes when the blue button is pressed, toggles the red LED, logs how fast it responded |
| Sensor | 40 | Every 10 ms, makes a fake descent sample (altitude, velocity, accel) and puts it in the queue |
| Telemetry | 32 | Takes samples off the queue and prints every 10th one as a CSV line |
| Monitor | 16 | Every 2 s, prints health: free memory, stack margins, dropped samples, missed deadlines |
| Heartbeat | 16 | Blinks the green LED every 0.5 s to show it's alive |

- **Queue:** Sensor never waits on it. If it's full the sample is dropped and counted, so the control loop never stalls because of logging.
- **Mutex:** only one task prints at a time, so lines never get mixed together.
- **Semaphore:** the button interrupt does almost nothing; it just wakes the Abort task, which runs immediately.

On RPL, the fake sensor becomes real drivers, Telemetry becomes the radio or SD logging, and the button becomes the real abort input.

## Build and flash

Install the tools from **Setup** first, then from `firmware/freertos-poc`:

```bash
cmake --preset Debug                 # first time only
cmake --build build/Debug            # build
STM32_Programmer_CLI -c port=SWD -w build/Debug/rpl_tst1.elf -v -rst   # flash
```

On macOS the flash tool lives inside `STM32CubeProgrammer.app/Contents/MacOs/bin/`. Or skip the terminal: in STM32CubeIDE, import it as a CMake project and press Run.

To see the output, plug in the board's USB and run:

```bash
pip3 install pyserial matplotlib
ls /dev/tty.usbmodem*                                      # find the port
python3 rpl_log.py capture /dev/tty.usbmodemXXXX run1.log  # Ctrl+C to stop
python3 rpl_log.py plot run1.log                           # makes run1.png
```

## What to test

| Test | How | Pass |
| --- | --- | --- |
| Build | Run the build commands | 0 errors, 0 warnings |
| Boot | Flash, press the black reset button | Startup banner prints; green LED blinks 1×/s; yellow toggles every 2 s |
| Telemetry | Capture 30 s, then plot | ~10 `T,` lines per second; plot shows a 10 m descent repeating about every 15 s |
| Health | Let it run 60 s, check the last `# MON` / `# STK` lines | `drop=0`, `ovr=0`, every `STK` value above 20 |
| Abort | Press the blue button 5 times | 5 `# EVENT` lines, `latency` 0–1 ms, red LED toggles each press |

If the red LED goes solid or blinks fast, the firmware crashed: solid means a task overflowed its stack, fast blinking means it ran out of memory.
