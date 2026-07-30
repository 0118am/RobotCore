# A-board external IMU firmware upgrade contract

This document defines the firmware-side work required by RobotCore's 60 Hz
state estimator. The currently discovered STM32 source is
`/home/nvidia/aCube_1`; it is a separate clean repository and is not modified
by RobotCore.

## Current behavior

The source identifies UT8/UART8 as a Bewei IMU connection:

- `Src/usart.c` configures UART8 as 9600 8N1.
- `application/hardware/Uart_receive_task.c` selects float gyro+acceleration
  output (`0x56 = 0x03`) and 10 Hz automatic output (`0x0C = 0x02`).
- Command `0x70` is parsed as six little-endian floats, but its source counter
  is ignored.
- `application/hardware/Usart_transmit_task.c` sends the latest value in
  telemetry frame 3. The telemetry scheduler runs frame 3 at about 20 Hz, so
  one 10 Hz IMU sample can be sent twice.

RobotCore will not treat periodic retransmission as new sensor data. Frame-3
word 7 must contain a changing source sample ID.

## Required target

| Item | Target |
| --- | --- |
| Sensor UART | 115200 baud, 8N1 |
| Sensor output | command `0x70`, float gyro xyz and acceleration xyz |
| Sensor rate | 100 Hz (`0x0C = 0x06`) |
| A-board to Jetson | UART6 at 115200 baud |
| Frame-3 delivery | once per new IMU sample, approximately 100 Hz |
| Frame-3 word 6 | nonzero only for a fresh, checksum-valid sample |
| Frame-3 word 7 | source counter/sample ID, unsigned modulo 65536 |

The six data values retain the existing scale:

- words 0--2: gyro xyz in centi-degrees/second;
- words 3--5: acceleration xyz in milli-g.

## Safe baud transition

Changing only the STM32 UART baud can permanently lose communication with a
sensor still stored at 9600. Implement one of these controlled procedures:

1. Preferred production behavior: probe/request at 115200 first; if no valid
   reply is received, fall back to 9600, send the Bewei saved baud command
   `77 05 00 0B 04 14`, verify its acknowledgement, reinitialize UART8 at
   115200, and verify command `0x70`.
2. Manufacturing procedure: configure and verify the sensor at 115200 with the
   vendor tool, then flash firmware whose UART8 default is 115200.

Do not save the 100 Hz automatic-output selection until its exact product model
has been confirmed. The Bewei protocol defines the selection, but supported
maximum rate is product-dependent.

## Scheduler and timestamp rules

- Parse and store the counter from bytes 24--27 of the `0x70` data area.
- Atomically copy gyro, acceleration, source counter, and receive tick.
- Enqueue frame 3 only when that counter changes. Do not publish the latest
  sample from an unrelated periodic telemetry loop.
- Keep frame 0/1/2 and PWM feedback at their existing operational rates unless
  bandwidth measurement requires a deliberate change.
- Measure UART8 checksum errors, duplicate counters, skipped counters, and
  sample age; expose them in diagnostics before closed-loop trials.

At 100 Hz, a 33-byte Bewei receive packet uses about 33 kbit/s on UART8. A
20-byte compact frame 3 at 100 Hz uses about 20 kbit/s on UART6. Both fit
115200 baud with margin, including existing traffic, but the complete UART6
schedule still needs a measured utilization and jitter check.

## Hardware acceptance

With thrusters disarmed and the vehicle stationary:

1. Observe at least 1,000 consecutive valid unique frame-3 samples.
2. Verify 95th-percentile interval at or below 12 ms and no interval above
   25 ms.
3. Verify no duplicate counter, and account for every counter skip.
4. Verify acceleration norm is close to local gravity and gyro is stationary.
5. Rotate each physical positive axis separately and confirm its ROS FLU sign.
6. Record gyro bias/noise, mounting RPY, and end-to-end arrival latency.
7. Run the RobotCore 60 Hz estimator check for at least 60 seconds.
