# Project Brief

## Hardware

- MCU: `STM32F407VET6`
- Inputs:
  - RC receiver for manual control
  - Orin for autonomous control
- Outputs:
  - PWM to ESC and steering servo for forward/reverse/turning control

## Deployment Note

This firmware was originally deployed on car A and is now being prepared for car
B. Wheelbase, track width, and related geometry parameters may need to be checked
and adjusted before rebuilding and flashing.

## Basic Workflow

1. Build the firmware source.
2. Replace or update the target firmware.
3. Use debug tools to verify RC input, Orin input, PWM output, and vehicle
   geometry parameters.
