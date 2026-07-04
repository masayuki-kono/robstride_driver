# Command-Tracking Test Results

Hardware-in-the-loop results showing that the actual motor state follows the
commands sent through this driver, in both velocity mode and CSP position
mode.

## Test setup

| Item | Value |
|------|-------|
| Motor | RobStride RS02 (motor id 127, factory default) |
| Transport | Official RobStride USB-CAN module (`AtSerialCanInterface`, `/dev/ttyUSB0`, 921600 baud) |
| Command / feedback rate | 100 Hz (every command's feedback response is recorded) |
| Load | No external load (bench test) |

Each command sent by `RobstrideMotor` is answered by a feedback frame
(communication type 2) carrying the measured position, velocity, torque and
temperature; the plots below record that feedback for every cycle. Positions
are made continuous with `PositionUnwrapper`.

## Velocity mode

Step profile 0 → +2 → +4 → −2 → 0 rad/s (via `SendVelocityCommand`, current
limit 10 A, acceleration 20 rad/s²):

![Velocity mode tracking](images/velocity_tracking.png)

Steady-state statistics per plateau (transients of 1.5 s excluded):

| Target [rad/s] | Mean actual [rad/s] | Max abs error [rad/s] |
|---------------|---------------------|------------------------|
| +2.0 | +1.998 | 0.31 |
| +4.0 | +4.005 | 0.26 |
| −2.0 | −2.003 | 0.30 |
| 0.0 | −0.002 | 0.25 |

- The mean velocity matches the target within 0.3 % on every plateau.
- Steps settle in well under 0.5 s (limited by the configured 20 rad/s²
  acceleration).
- The residual ±0.3 rad/s band is measurement ripple of the velocity
  estimate, not a control offset — the resulting position ramps are straight.

## CSP position mode

Step profile 0 → +π → −π → 0 rad (via `SendPositionCspCommand` semantics:
`limit_spd` = 4 rad/s, then `loc_ref` steps). The motor was zeroed with
`SetMechanicalZero()` before the run:

![CSP position mode tracking](images/position_tracking.png)

Steady-state statistics per plateau (transients of 2 s excluded):

| Target [rad] | Mean actual [rad] | Max abs error [rad] |
|--------------|-------------------|----------------------|
| +3.142 | +3.1405 | 0.0017 |
| −3.142 | −3.1408 | 0.0013 |
| 0.000 | −0.0002 | 0.0006 |

- Steady-state position error is below 0.002 rad (≈ 0.1°).
- During the moves the velocity is bounded by the configured 4 rad/s limit
  (lower plot), giving a constant-speed ramp between targets.

## Reproducing

The data was collected with a small program built on the public API — the
same call sequence as [examples/velocity_control.cpp](../examples/velocity_control.cpp)
(velocity mode) and `SetRunMode(kPositionCsp)` + `WriteParam(kLimitSpd/kLocRef)`
(position mode), logging the `Feedback` returned by each command to CSV at
100 Hz.
