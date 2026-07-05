# RobStride Private CAN Protocol

Summary of the RobStride private CAN protocol as implemented by this driver. Authoritative source: **RS02 User Manual, chapter 4 "Driver protocol and instructions"** (`third_party/Product_Information`, to be consulted from the vendor's download center once the submodule is removed).

## Supported protocol variants

The motor supports three protocols on the CAN bus. They are selected with
communication type 25 (switch protocol); a power cycle is required after switching.

| Protocol | Manual | Summary | This driver |
|----------|--------|---------|-------------|
| Private protocol | Ch. 4 "Driver protocol and instructions" | RobStride proprietary protocol. 29-bit extended frames. **Power-on default**. Parameter read/write and all control modes (operation / PP position / CSP position / velocity / current) | **Implemented** |
| CANopen | Ch. 5 "Explanation of Canopen Communication Protocol Types" | Standard higher-layer protocol (OSI layer 7) using 11-bit CAN IDs. EDS files available from the vendor | Not implemented |
| MIT Protocol | Ch. 6 "MIT Communication Protocol Description" | Open motor-control protocol derived from MIT Mini Cheetah; used by projects such as OpenArm | Not implemented |

This driver implements the **private protocol only** (the sections below describe it).

### Why only the private protocol

- It is the power-on default — no protocol-switch frame or power cycle needed.
- It covers parameter read/write and every control mode; nothing essential is
  missing for typical robot integration.
- The official ROS samples also use the private protocol over SocketCAN.

Switching to CANopen or MIT may become necessary later (e.g. mixing with
CANopen devices or reusing MIT-protocol software), but for initial Linux
integration and feasibility work the private protocol is sufficient.

## Physical layer

- CAN 2.0B, **29-bit extended frames**, DLC always 8
- Bit rate 1 Mbps (default; changeable via communication type 23)

## 29-bit identifier layout

### Host → motor (general form)

| Bits 28–24 | Bits 23–8 | Bits 7–0 |
|------------|-----------|----------|
| communication type | data area 2 (usually host CAN id in bits 15–8) | target motor CAN id |

### Motor → host (feedback, type 2)

| Bits 28–24 | 23–22 | 21–16 | 15–8 | 7–0 |
|------------|-------|-------|------|-----|
| 0x02 | mode status | fault bits | motor CAN id | host CAN id |

Mode status: 0 = reset, 1 = calibration, 2 = run.

Fault bits (1 = fault): bit16 undervoltage, bit17 three-phase overcurrent, bit18 overtemperature, bit19 magnetic-encoding fault, bit20 stall/overload, bit21 uncalibrated.

## Communication types

| Type | Name | Direction | Notes |
|------|------|-----------|-------|
| 0x00 | Get device id | H→M | reply carries 64-bit MCU uid |
| 0x01 | Operation (motion) control | H→M | torque in id bits 23–8; pos/vel/Kp/Kd in data |
| 0x02 | Feedback | M→H | response to most commands |
| 0x03 | Enable | H→M | data all zero |
| 0x04 | Stop | H→M | data[0]=1 clears a latched fault |
| 0x06 | Set mechanical zero | H→M | data[0]=1; motor must be stopped |
| 0x07 | Set motor CAN id | H→M | new id in id bits 23–16; effective immediately |
| 0x11 (17) | Read single parameter | H→M | index little-endian in data[0..1] |
| 0x12 (18) | Write single parameter | H→M | volatile; save with type 22 |
| 0x15 (21) | Fault feedback | M→H | 32-bit fault + warning values |
| 0x16 (22) | Save data | H→M | persists 0x20xx-backed parameters |
| 0x17 (23) | Set baud rate | H→M | effective after power cycle |
| 0x18 (24) | Active reporting on/off | H→M | periodic type-2 frames, default 10 ms |
| 0x19 (25) | Switch protocol | H→M | 0 private / 1 CANopen / 2 MIT; power cycle |

## Feedback data field (type 2)

Big-endian 16-bit values, linearly scaled:

| Bytes | Value | RS02 range |
|-------|-------|------------|
| 0–1 | angle | −4π … +4π rad |
| 2–3 | angular velocity | −44 … +44 rad/s |
| 4–5 | torque | −17 … +17 Nm |
| 6–7 | temperature | value = °C × 10 |

Scaling: `u16 = (x − min) · 65535 / (max − min)`, clamped.

## Motion control command (type 1)

- Identifier bits 23–8: torque feed-forward, scaled to the ±torque range
- Data (big-endian u16): bytes 0–1 target angle, 2–3 target velocity, 4–5 Kp (0…500), 6–7 Kd (0…5)
- Control law: `t_ref = Kd·(v_set − v_actual) + Kp·(p_set − p_actual) + t_ff`

## Parameter read/write (types 17/18)

Request/response data layout:

| Bytes | Content |
|-------|---------|
| 0–1 | parameter index, **little-endian** |
| 2–3 | 0x0000 |
| 4–7 | value, **little-endian** (float32 IEEE-754, or uint8 in byte 4) |

### Parameter index table (0x7000 family)

| Index | Name | Type | Access | Description |
|-------|------|------|--------|-------------|
| 0x7005 | run_mode | uint8 | RW | 0 operation control, 1 position PP, 2 velocity, 3 current, 4 zero, 5 position CSP |
| 0x7006 | iq_ref | float | RW | current-mode Iq command [A] |
| 0x700A | spd_ref | float | RW | velocity-mode speed command [rad/s] |
| 0x700B | limit_torque | float | RW | torque limit [Nm] |
| 0x7010 | cur_kp | float | RW | current loop Kp |
| 0x7011 | cur_ki | float | RW | current loop Ki |
| 0x7014 | cur_filt_gain | float | RW | current filter gain |
| 0x7016 | loc_ref | float | RW | position command [rad] |
| 0x7017 | limit_spd | float | RW | CSP position-mode speed limit [rad/s] |
| 0x7018 | limit_cur | float | RW | velocity/position-mode current limit [A] |
| 0x7019 | mechPos | float | R | load-side angle [rad] |
| 0x701A | iqf | float | R | filtered Iq [A] |
| 0x701B | mechVel | float | R | load-side speed [rad/s] |
| 0x701C | VBUS | float | R | bus voltage [V] |
| 0x701E | loc_kp | float | RW | position loop Kp |
| 0x701F | spd_kp | float | RW | velocity loop Kp |
| 0x7020 | spd_ki | float | RW | velocity loop Ki |
| 0x7021 | spd_filt_gain | float | RW | velocity filter gain |
| 0x7022 | acc_rad | float | RW | velocity-mode acceleration [rad/s²] |
| 0x7024 | vel_max | float | RW | PP position-mode speed [rad/s] |
| 0x7025 | acc_set | float | RW | PP position-mode acceleration [rad/s²] |
| 0x7026 | EPScan_time | uint16 | RW | active-report interval (1 = 10 ms, +1 = +5 ms) |
| 0x7028 | canTimeout | uint32 | RW | CAN watchdog, 20000 = 1 s, 0 = disabled |
| 0x7029 | zero_sta | uint8 | RW | power-on position range: 0 → 0…2π, 1 → −π…π |

## Control-mode sequences (per manual §4.3)

Mode changes must be done while the motor is stopped.

| Mode | Sequence |
|------|----------|
| Operation control | enable (3) → motion command (1) |
| Velocity | write run_mode=2 (18) → enable (3) → write limit_cur → write acc_rad → write spd_ref |
| Position CSP | write run_mode=5 (18) → enable (3) → write limit_spd → write loc_ref |
| Position PP | write run_mode=1 (18) → enable (3) → write vel_max → write acc_set → write loc_ref |
| Current | write run_mode=3 (18) → enable (3) → write iq_ref |
| Stop | stop frame (4) |

## Per-model scaling ranges

Used for type-1 commands and type-2 feedback:

| Model | Position [rad] | Velocity [rad/s] | Torque [Nm] | Kp max | Kd max |
|-------|---------------|------------------|-------------|--------|--------|
| RS00 | ±4π | ±50 | ±17 | 500 | 5 |
| RS01 | ±4π | ±44 | ±17 | 500 | 5 |
| **RS02** | **±4π** | **±44** | **±17** | **500** | **5** |
| RS03 | ±4π | ±50 | ±60 | 5000 | 100 |
| RS04 | ±4π | ±15 | ±120 | 5000 | 100 |
| RS05 | ±4π | ±33 | ±17 | 500 | 5 |
| RS06 | ±4π | ±20 | ±36 | 5000 | 100 |

RS02 values confirmed by the RS02 User Manual (P ±12.57 rad, V ±44 rad/s, T ±17 Nm, Kp 0–500, Kd 0–5). Other rows originate from vendor sample code.

## Worked example (from the manual, §4.1.14)

Reading `loc_kp` (0x701E) from motor 0x7F, host 0xFD:

```
request : id=0x11 00FD 7F  data = 1E 70 00 00 00 00 00 00
response: id=0x11 007F FD  data = 1E 70 00 00 00 00 F0 41   (0x41F00000 = 30.0f)
```

## Serial framing of the RobStride USB-CAN module

The official USB-CAN module carries the same CAN frames over a 921600-baud serial link (CH340), wrapping each frame as:

```
"AT" | 4-byte big-endian ((29-bit id << 3) | 0x4) | DLC | data (DLC bytes) | "\r\n"
```

Worked example from the manual (§3.3.5) — writing parameter 0x7005 to motor 0x01 from host 0xFD (CAN id `0x1200FD01`):

```
41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
```

`0x9007E80C = (0x1200FD01 << 3) | 0x4`. Frames received from the module use the same format. Implemented by `AtSerialCanInterface` (`at_serial::encode_frame` / `at_serial::FrameParser`).
