# A-board UART6 protocol v2

This is the only production propulsion transport between RobotCore and aCube.
It deliberately carries eight logical thrusters; aCube maps logical channel
`0..7` to physical PWM `8..15`. PWM `0..7`, the legacy onboard mixer, CAN
motion commands, and motion commands received on other UARTs are not control
authorities.

The board has no voltage or temperature measurement module. Those values are
not present on the wire and must never participate in arming or failsafe logic.
This pool hardware version also has no depth or altitude scalar path. With the
pool floor as the map origin, the sole vertical state is the FLU
`BodyState.pose.position.z` coordinate.

All multibyte integers are little-endian. CRC is CRC-16/CCITT-FALSE with
polynomial `0x1021` and initial value `0xffff`, stored low byte first.

## Command, Jetson to aCube

The bridge sends one 34-byte frame at 50 Hz. It does not send per-command
deadlines or per-source leases.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | `0xff` |
| 1 | 1 | command type `0xfc` |
| 2 | 1 | protocol version `2` |
| 3 | 1 | flags; bit 0 is output enable |
| 4 | 4 | current nonzero board boot-ID challenge |
| 8 | 4 | nonzero host-process session ID |
| 12 | 4 | monotonically increasing sequence |
| 16 | 16 | eight signed PWM offsets in microseconds |
| 32 | 2 | CRC over bytes 0 through 31 |

Shared command golden vector (`boot=0x01020304`, `session=0x12345678`,
`sequence=0x9abcdef0`, offsets `[-100,-1,0,1,25,50,99,100]`):

```text
fffc02010403020178563412f0debc9a9cffffff000001001900320063006400c3dc
```

A disabled frame whose boot ID matches the board's current status challenge
establishes or changes the single control session and applies neutral. Until a
valid status has supplied that challenge, RobotCore sends only boot ID zero and
disabled output; aCube rejects it. An enabled frame is accepted only for the
current boot challenge and established session, and only when its sequence is
newer. A boot mismatch, duplicate or old sequence, CRC failure, or enabled frame
from another session does not refresh the board watchdog. A boot or session
mismatch also makes the outputs neutral, so restarting either processor
requires a fresh disabled handshake plus an explicit disarm and re-arm.

RobotCore independently turns a stale ROS producer command into a disabled
frame after 150 ms. aCube has one transport watchdog at 250 ms. These are the
two necessary fault-containment boundaries; there are no overlapping leases
inside the UART protocol.

## Status/ACK, aCube to Jetson

aCube sends one 48-byte frame at 20 Hz.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | `0xff` |
| 1 | 1 | status type `0xfd` |
| 2 | 1 | protocol version `2` |
| 3 | 1 | session/output/failsafe flags |
| 4 | 4 | board monotonic tick in milliseconds |
| 8 | 4 | nonzero per-boot STM32 hardware-RNG challenge |
| 12 | 4 | retained active/session-baseline ID |
| 16 | 4 | last accepted sequence |
| 20 | 4 | last sequence physically latched at the synchronized PWM update boundary |
| 24 | 2 | accepted-command age in milliseconds, saturated at 65535 |
| 26 | 1 | safety reason |
| 27 | 1 | RCC reset-cause flags, bits 31 through 24 |
| 28 | 2 | saturated command CRC-error count |
| 30 | 16 | reported PWM microseconds for physical PWM 8 through 15 |
| 46 | 2 | CRC over bytes 0 through 45 |

Shared status golden vector:

```text
fffd0203e110000004030201ddccbbaa6500000064000000110003a50700dc05dd05de05df05e005e105e205e3053b20
```

The ACK is evidence that all eight preloaded compares crossed the synchronized
TIM4/TIM5 hardware update boundary, not merely UART receipt or a RAM write.
`outputs` reports what the previous update boundary actually latched, so the
bounded transition to neutral may truthfully report outputs and failsafe at the
same time. RobotCore treats a stale status, an unacknowledged boot-bound
session, or a board failsafe flag as unsafe.

## Bandwidth and jitter budget

UART6 is 115200 baud, 8N1, so each direction can carry 11520 bytes/s.

| Direction | Traffic | Bytes/s | Link use |
| --- | --- | ---: | ---: |
| Jetson to aCube | 34 B command at 50 Hz | 1700 | 14.8% |
| aCube to Jetson | 27 B IMU at 100 Hz + 48 B status at 20 Hz | 3660 | 31.8% |

When IMU and status become ready together, their 75 bytes serialize in 6.51
ms. This remains below the 10 ms IMU sample period. CRC failures, command age,
boot/reset changes, and applied-sequence lag are exported as diagnostics and
must be checked during full-power pool acceptance.

Protocol v2 is an atomic firmware/host cutover. The production parser does not
fall back to additive-checksum `0xf9`, `0xfa`, or `0xfb` propulsion frames.
