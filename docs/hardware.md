# Hardware Setup

This document describes the hardware configuration assumed by `robstride_driver`. The primary target is the **RobStride RS02** quasi-direct-drive motor module; other RobStride models share the same electrical interface and CAN protocol.

## Assumed configuration

```
+------------+   USB    +-----------------+   CAN bus (1 Mbps)   +--------+     +--------+
| Linux host |----------| USB-CAN adapter |----------------------|  RS02  |-...-|  RS02  |
+------------+          +-----------------+  CAN_H / CAN_L       | id=1   |     | id=N   |
                              [120R]                             +--------+     +--------+
                                                                                  [120R]
```

Two kinds of USB-CAN adapters are supported (see below):

- a **SocketCAN-compatible adapter** (recommended), used via `SocketCanInterface`
- the **official RobStride USB-CAN module**, used via `AtSerialCanInterface`

## RS02 motor

Key electrical specifications (see the official RS02 User Manual in full):

| Item | Value |
|------|-------|
| Rated voltage | 48 VDC (operating range 24–60 VDC) |
| Rated / peak torque | 6 Nm / 17 Nm |
| No-load speed | 410 rpm (approx. 43 rad/s) |
| CAN bit rate | 1 Mbps (fixed default) |
| Encoder | 14-bit absolute |
| Gear ratio | 7.75 : 1 |

### Connector

The motor has a single XT30PB(2+2) connector carrying power and CAN:

| Pin | Function |
|-----|----------|
| 1 | Power + |
| 2 | Power − |
| 3 | CAN_L |
| 4 | CAN_H |

Mating connector: AMASS XT30(2+2)-F.G.B.

## USB-CAN adapter

### SocketCAN adapters (recommended)

Any adapter with a mainline Linux SocketCAN driver works, for example:

- **Canable / candleLight firmware** (`gs_usb` driver) — plug and play
- Adapters based on `slcan` (serial-line CAN) also work but add latency

Use `SocketCanInterface` with these adapters. Kernel-side receive filtering (`set_motor_id_filter`) is available.

### RobStride official USB-CAN module

The RobStride official USB-CAN module (the adapter shipped for the vendor's Windows tuning tool) is a CH340 USB-serial bridge in front of a GD32 MCU. It does **not** expose a SocketCAN interface; instead it converts CAN frames to a serial protocol:

- serial link: 921600 baud, 8N1
- frame format: `"AT"` + 4-byte big-endian `((can_id << 3) | 0x4)` + DLC + data + `"\r\n"`
- works in AT mode by default (DIP switch 1 OFF); DIP switch 2 ON connects a 120 Ω terminator

Use `AtSerialCanInterface("/dev/ttyUSB0")` with this module. The CH340 driver is in the mainline kernel, so the module shows up as `/dev/ttyUSB*` automatically; add your user to the `dialout` group for access.

## Bus wiring

- Use twisted-pair wire for CAN_H/CAN_L.
- Terminate **both ends** of the bus with 120 Ω (many USB-CAN adapters have a switchable terminator; the far-end motor needs one too).
- Keep stubs short; daisy-chain multiple motors rather than star-wiring.
- The bit rate is 1 Mbps — keep total bus length within ~25 m.

## Multiple motors on one bus

Each motor needs a unique CAN id (factory default `0x7F` = 127).

To assign ids, connect **one motor at a time** and either:

- use the vendor's Windows tuning tool, or
- use this driver's communication type 7 frame (`make_set_can_id_frame`), which changes the id immediately. Persist it afterwards with the data-save frame (type 22) if required by your firmware version.

The driver-side receive filter (`SocketCanInterface::set_motor_id_filter`) matches the motor id carried in bits 15–8 of feedback frames, so per-motor sockets can coexist on one interface.

## Power-up behavior

- The motor powers up in **operation control (MIT) mode**, disabled.
- Optionally configure the CAN watchdog (`canTimeout`, parameter 0x7028; 20000 = 1 s) so the motor disables itself if the host stops sending commands.
