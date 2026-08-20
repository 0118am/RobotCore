# Aquaboard external IMU firmware upgrade contract

This document records the firmware/host protocol used by RobotCore's 60 Hz
state estimator. The STM32 source is the separate repository
`/home/nvidia/aquaboard`; it must be flashed atomically with the matching C++
host bridge.

## Superseded baseline

The source identifies UT8/UART8 as a Bewei IMU connection:

- `Src/usart.c` configures UART8 as 9600 8N1.
- `application/hardware/Uart_receive_task.c` selects float gyro+acceleration
  output (`0x56 = 0x03`) and 10 Hz automatic output (`0x0C = 0x02`).
- Command `0x70` is parsed as six little-endian floats, but its source counter
  is ignored.
- `application/hardware/Usart_transmit_task.c` sends the latest value in
  telemetry frame 3. The telemetry scheduler runs frame 3 at about 20 Hz, so
  one 10 Hz IMU sample can be sent twice.

RobotCore does not treat this legacy frame-3 path as a production IMU input.

## Required target

| Item | Target |
| --- | --- |
| Sensor UART | 115200 baud, 8N1 |
| Sensor output | connected VG/AH/MINS mode `0x06`, 48-byte `0x59` float packet; IMU-family `0x70` also accepted |
| Sensor rate | 100 Hz (`0x0C = 0x06`) |
| Aquaboard to Jetson | UART6 at 115200 baud |
| Frame-4 delivery | once per new IMU sample, approximately 100 Hz |
| Integrity | version 1, valid flag, CRC16-CCITT |
| Time/sequence | uint32 extension of the source counter and uint32 MCU sample tick |

The versioned 27-byte wire layout is:

`FF F8 04 | version=1 | flags | uint32 counter | uint32 tick_ms | 3×int16 gyro | 3×int16 accel | CRC16-CCITT`

The six data values use:

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

- For the connected sensor, decode the packed-BCD `000`--`255` counter from
  bytes 43--44 of the 48-byte `0x59` packet and extend its 8-bit wrap to
  uint32. The IMU-family 33-byte `0x70` packet uses bytes 28--29 similarly.
- Atomically copy gyro, acceleration, source counter, and receive tick.
- Enqueue frame 4 only when that counter changes. Do not publish the latest
  sample from an unrelated periodic telemetry loop.
- Production UART6 emits only frame 4 at up to 100 Hz and protocol-v2 status at
  20 Hz; legacy rotating frame 0/1/2 and `FF FB` feedback are removed.
- Measure UART8 checksum errors, duplicate counters, skipped counters, and
  sample age; expose them in diagnostics before closed-loop trials.

At 100 Hz, the connected sensor's 48-byte receive packet uses about 48 kbit/s
on UART8. A
27-byte frame 4 at 100 Hz uses about 27 kbit/s on UART6. Together with the
48-byte status at 20 Hz, UART6 TX uses 31.8% of an 8N1 115200-baud line; a
coincident 75-byte burst serializes in 6.51 ms.

## Hardware acceptance

With thrusters disarmed and the vehicle stationary:

1. Observe at least 1,000 consecutive valid unique frame-4 samples.
2. Verify 95th-percentile interval at or below 12 ms and no interval above
   25 ms.
3. Verify no duplicate counter, and account for every counter skip.
4. Verify acceleration norm is close to local gravity and gyro is stationary.
5. Rotate each physical positive axis separately and confirm its ROS FLU sign.
6. Record gyro bias/noise, mounting RPY, and end-to-end arrival latency.
7. Run the RobotCore 60 Hz estimator check for at least 60 seconds.

## Historical 2026-08-02 baseline (superseded)

The firmware built on that date configured UART8 for 115200 baud and requested
100 Hz float output, forwarded each newly parsed sample as frame 4, extended
the sensor's BCD counter, included the MCU acquisition tick, and protected the
payload with CRC16-CCITT. Its Release image SHA256
`57b7df80d412125b87ca93b9d343c05494c57f3c523ecf3044d2d98a0ebf6871`
was flashed and independently read back byte-for-byte on 2026-08-02. A
debugger-side run observed two samples spanning multiple source-counter wraps:
frame count `1726 -> 2737` and extended sample ID `1776 -> 2787`, both `+1011`,
with zero UART8 checksum errors and zero stack overflows. That hash predates
the protocol-v2 session/ACK, synchronized PWM latch, and task cleanup and must
not be treated as the current production image.

The current Aquaboard and RobotCore sources must be built, flashed, and deployed as
one atomic protocol-v2 cutover. No claim is made here that the current image
has been flashed. After deployment, complete estimator calibration, timing,
noise, physical-axis, brownout, and full-thrust acceptance before arming in
water.
