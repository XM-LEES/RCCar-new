# Module Usage Audit

This audit captures the cleanup state after removing unused source modules from
the current `cleanup/project-structure-from-5233fdc` branch.

## Active Runtime Modules

- `SerialControlTask` is created by `MX_FREERTOS_Init()` and consumes
  `g_xQueueROSserial`, which is fed by the UART4 receive callback.
- `ServoBasic_Task` is created by `MX_FREERTOS_Init()` and is the active
  arbitration/output loop for Orin PWM targets, RC passthrough, and guard
  override.
- `servo_rc_capture.c` is active through TIM4 CH1/CH2/CH3 input capture and
  `HAL_TIM_IC_CaptureCallback()`.
- `hall_speed.c` is active through `HallSpeed_Init()` and
  `HAL_GPIO_EXTI_Callback()`.
- `RobotDataTransmitTask` is created by `MX_FREERTOS_Init()` and sends UART4
  base telemetry.
- `show_task`, `ImuTask`, and `RGBControl_task` are created by
  `MX_FREERTOS_Init()` and remain active support tasks.

## Removed In This Cleanup

| Module | Evidence before cleanup | Cleanup result |
| --- | --- | --- |
| Bluetooth/App control | `BlueToothControlTask` was not created and `g_xQueueBlueTooth` was never allocated. | Deleted Bluetooth/App task files, removed UART2 App receive feed, and removed Keil entries. |
| USB HID gamepad control | `MX_USB_HOST_Init()` was declared but not called, and USB gamepad sources were only project entries. | Deleted `USB_HOST/` and `bsp_gamepad.*`, removed HCD/USB Host project entries and include paths. |
| AutoRecharge | `AutoRechargeTask` was not created and `g_xQueueAutoRecharge` was never allocated. | Deleted task/header, removed charge commands, charge telemetry extension, and charge RGB states. |
| Ranger ultrasonic | Ranger task path was not started while telemetry/RGB still read passive `RangerHAL_*` state. | Deleted Ranger module/header, removed ranger telemetry extension, RGB warning, and `RangerAvoidEN`. |
| `reportErr_task.c` | The task was not created and depended on Bluetooth/App, AutoRecharge, old CAN servo diagnostics, and removed command sources. | Deleted the stale report task and removed its Keil entry. |
| Hall debug telemetry | The 32-byte debug frame was diagnostic-only and conflicted with the fixed ROS UART4 parser contract if mixed into the stream. | Deleted the debug frame path plus IRQ/Callback/accepted-edge counters; kept `g_hall_speed_state` for Keil Watch/OLED diagnosis. |

## Still Present But Not Primary Motion Control

| Module | Current evidence | Follow-up constraint |
| --- | --- | --- |
| `RobotControl_task.c` | Kept for utility functions used by serial control: reset command, debug level parsing, limit helpers, and command source storage for legacy RC joystick code. The task itself is not created. | Consider splitting these utilities only when cleaning the remaining legacy control compatibility layer. |
| `rc_joystick.c` | Present in the Keil project, but no FreeRTOS task starts this path. | This is not the active RC receiver takeover path. Active RC takeover uses `servo_rc_capture.c` and `ServoBasic_Task()`. |
| `bsp_ServoDrive.c` | Present in the Keil project. `g_xQueueCANopenCallback` is still not allocated in current startup. | Delete only after old CAN servo debug/config workflows are confirmed unused. |

## Follow-up Order

1. Rebuild with Keil to confirm the pruned project file compiles on the target
   toolchain.
2. Audit the remaining legacy control compatibility layer:
   `RobotControl_task.c`, `rc_joystick.c`, `bsp_ServoDrive.c`, and related CAN
   queue placeholders.
3. Recheck `WHEELTEC.ioc` before any future CubeMX regeneration; it is now
   aligned with this cleanup state and no longer configures USB Host,
   USB OTG FS, Bluetooth/App USART2, Ranger TIM2/TIM3, or Ranger GPIO.
