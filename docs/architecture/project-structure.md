# Project Structure

This describes the current Ackermann-only firmware branch after pruning unused
Bluetooth/App, USB HID gamepad, AutoRecharge, Ranger, ICM20948/IMU, RGB runtime,
and CAN runtime source paths.

## Source Layout

- `Core/` contains STM32Cube-generated startup, peripheral init, interrupt
  files, and FreeRTOS task creation.
- `WHEELTEC_APP/` contains vehicle behavior: serial command parsing, Ackermann
  PWM control, RC capture, hall speed feedback, telemetry, and OLED display.
- `WHEELTEC_BSP/` contains board support source for OLED, ADC, DWT, CAN,
  flash, RGB, buzzer, software I2C, and older CAN servo drive helpers. Only
  the subset needed by the current APP is compiled by the Keil project.
- `Drivers/` and `Middlewares/` are vendor HAL/CMSIS/FreeRTOS libraries.
  STM32 USB Host middleware remains as vendor code, but it is no longer in the
  Keil build.
- `MDK-ARM/` contains the Keil project and generated build/debug artifacts.
  Build artifacts are intentionally out of scope for the current cleanup pass.
- `docs/` contains maintained project notes, protocol references,
  investigation notes, vendor manuals, and archived stale references.
- `舵机基本控制/` is vendor/source-side material kept for reference until a
  separate asset cleanup pass decides its final home.
- `更新记录.txt` is a vendor update note kept at the repository root because
  the Keil project references it by relative path.

## Runtime Startup

`Core/Src/main.c` performs the hardware setup and starts FreeRTOS:

1. Initializes GPIO, DMA, UART1 TX debug, UART4 ROS, TIM4/6/8/9/11, and ADC.
2. Calls `ServoBasic_Init()`, `DWT_Init()`, `HallSpeed_Init()`, and
   `ADC_Userconfig_Init()`.
3. Starts TIM6 for runtime stats and initializes the OLED.
4. Detects the C63x hardware version and adjusts pin setup for v1.0 boards.
5. Starts TIM4 input capture on CH1/CH2/CH3 for RC throttle, steering, and
   guard input.
6. Starts byte-wise UART receive interrupts on UART4 only.
7. Calls `Robot_Select()` to load the current Ackermann geometry constants.
8. Calls `MX_FREERTOS_Init()` and starts the scheduler.

`Core/Src/freertos.c` currently creates these application tasks:

- `ServoBasic_Task` at 50 Hz for RC/Orin arbitration and PWM output.
- `show_task` at 10 Hz for OLED Ackermann status.
- `SerialControlTask` for UART4 command parsing.
- `RobotDataTransmitTask` at 20 Hz for UART4 telemetry and UART1 debug output.
- `StartInitTask`, which only emits a buzzer startup tip and exits.

## Current Control And Feedback Paths

The active autonomous control path is:

```text
UART4 byte IRQ
  -> g_xQueueROSserial
  -> SerialControlTask()
  -> ServoBasic_UpdateFromOrin(vx, vy, vz, flag_stop)
  -> ServoBasic_Task()
  -> TIM8 CH1/CH2 PWM
  -> PC6 ESC, PC7 steering servo
```

The active RC path is:

```text
TIM4 CH1/CH2/CH3 input capture
  -> HAL_TIM_IC_CaptureCallback()
  -> ServoRC_IC_CaptureCallback()
  -> ServoBasic_Task()
  -> RC passthrough or guard override
  -> TIM8 CH1/CH2 PWM
```

The active speed feedback path is:

```text
PE13/PE14 hall GPIO
  -> HAL_GPIO_EXTI_Callback()
  -> HallSpeed_OnCountEvent()
  -> ServoBasic_GetOrinFeedback()
  -> RobotDataTransmitTask()
  -> UART4 24-byte base telemetry frame
```

Support paths that are active but not primary motion control:

- `show_task` displays Ackermann mode, PWM, RC override, guard, voltage, and
  geometry values.
- `RobotDataTransmitTask` keeps the 24-byte ROS frame; bytes `8..19` are IMU
  placeholders fixed to zero because ICM20948 support was removed.
- `RobotDataTransmitTask` sends the same 24-byte base frame on UART4 and UART1
  TX debug output when `DebugLevel == 0`; USART1 RX is not started.

## Removed Feature Paths

These older feature paths are intentionally removed from source and from the
Keil project:

- Bluetooth/App control: `BlueTooth_task.c`, `appshow_task.c`, UART2 App receive
  queue/feed.
- USB HID gamepad control: `USB_HOST/`, `bsp_gamepad.*`, HCD/USB Host compile
  entries and include paths.
- AutoRecharge: `AutoRecharge_task.c`, charge command handling, charge telemetry
  extension, and charge RGB states.
- Ranger ultrasonic ranging: `sensor_ranger.c`, ranger telemetry extension,
  ranger RGB warning, `RangerAvoidEN`, TIM2/TIM3 capture setup, and ultrasonic
  trigger/echo GPIO.
- ICM20948 IMU: `imu_task.c`, `bsp_icm20948.c`, and ICM20948 register/header
  files. The ROS 24-byte frame length is unchanged; the former IMU bytes are
  fixed zero placeholders.
- RGB APP runtime: `RGBStripControl_task.*` and runtime RGB command state. TIM9
  and TIM11 still exist in the CubeMX/Core setup; RGB BSP source is retained but
  no longer compiled into the current Keil target.
- Legacy RC joystick and RobotControl compatibility: `rc_joystick.*` and
  `RobotControl_task.*`; serial reset/log helpers now live in
  `SerialControl_task.c`.
- Legacy CAN servo drive build entry: `bsp_ServoDrive.c` is no longer compiled
  by the Keil project, but the BSP source is kept for historical recovery.
- CAN current-target entry: `MX_CAN1_Init()` / `MX_CAN2_Init()`, CAN IRQ
  handlers, the HAL CAN module switch, Keil `can.c` /
  `stm32f4xx_hal_can.c` entries, and `WHEELTEC.ioc` CAN configuration are
  removed. `Core/Src/can.c`, `Core/Inc/can.h`, and BSP CAN sources remain only
  as historical recovery material.
- USART3/RS485 current-target entry: `MX_USART3_UART_Init()`, USART3 RX DMA,
  USART3 IRQ handlers, PB10/PB11 USART3 pin config, and `WHEELTEC.ioc` USART3
  config are removed because no current APP task consumes RS485 bytes.
- USART1 RX receive path: the unused byte receive buffer and receive restart
  path are removed. USART1 remains TX-only debug output for the duplicated
  24-byte telemetry frame.
- Dormant BSP compile entries: `bsp_RGBLight.c`, `bsp_siic.c`, `bsp_can.c`,
  `bsp_key.c`, `bsp_RTOSdebug.c`, and `bsp_led.c` are no longer compiled by the
  current Keil target. Their source files remain under `WHEELTEC_BSP/`.
- Hall debug telemetry: removed the 32-byte Hall debug frame and
  IRQ/Callback/accepted-edge debug counters so UART4 stays on the fixed ROS
  frame contract.

## Current Keil BSP Boundary

Compiled BSP files are limited to:

- `bsp_buzzer.c`
- `bsp_dwt.c`
- `bsp_oled.c`
- `bsp_flash.c`
- `bsp_adc.c`

Dormant flash-backed parameter storage is intentionally kept available for
future power-off persistence. The current APP no longer reads the old default
speed or line-diff parameters at startup.

TIM9/TIM11 RGB PWM pin setup and PB6/PB7 IIC GPIO setup remain in Core as
board-reserved hardware configuration. Their BSP runtime sources are not in the
current Keil target, so they do not create APP tasks or control behavior.

Dormant BSP source is kept for board recovery or future feature work, but must
be re-added to the Keil project together with the APP owner that calls it.

The active RC receiver takeover path remains separate from the removed USB HID
gamepad path.

## Cleanup Rules

- Do not delete a file only because its task is not currently created; Keil
  project references, globals, callback hooks, or debug workflows may still
  depend on it.
- Treat `Core/Src/freertos.c`, `Core/Src/main.c`, interrupt callbacks, and the
  Keil project file as the first evidence sources for whether code is active.
- Split shared globals or utility functions before deleting old task modules
  that still define data used by active code.
- Keep dormant BSP source out of the Keil build when the current APP does not
  allocate its queues or call its runtime entrypoints.
- Keep vendor libraries and build artifacts out of source cleanup commits.
- `WHEELTEC.ioc` is aligned with the cleanup state: USB Host, USB OTG FS,
  Bluetooth/App USART2, USART2 TX DMA, Ranger TIM2/TIM3, Ranger GPIO, CAN1/2,
  CAN NVIC, CAN pins, USART3/RS485, and USART1 RX are no longer configured.
  Keep UART4 telemetry at 24 bytes and ROS commands at 11 bytes unless the
  upper-computer parser is changed at the same time.
- Do not remove `DWT` as part of IMU cleanup; hall speed timing still depends on
  the Cortex-M cycle counter.
