# EUP Firmware Skeleton

This directory reserves the low-level RoboMaster Aboard or replacement-board
firmware boundary.

Target responsibilities:

- Validate command packets.
- Map 8 normalized thruster commands to PWM.
- Maintain heartbeat and failsafe.
- Handle estop.
- Return board status.

Phase 0 must confirm the exact Aboard model, STM32 chip, PWM channel count,
signal level, transport, and flashing method before board-specific code is
added.
