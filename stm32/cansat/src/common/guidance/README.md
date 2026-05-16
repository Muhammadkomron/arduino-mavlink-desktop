# Guidance

The guidance module steers an autonomous parafoil toward a fixed target
landing point. It runs at 5 Hz on the flight controller, reads sensor state,
and emits left/right brake-line pull commands that are pushed to the steering
servos on PE11/PE13.

---

## The physical system

A small parafoil (canopy + payload, ~1.5 kg) is air-dropped from altitude.
Two servos on the payload each spool one half of the canopy's brake lines.
Pulling a line shortens that side's trailing edge, increasing drag on that
half-wing, banking the canopy toward the pulled side.

There is **no speed-bar, no throttle, no rudder** — only differential
braking. Forward speed and sink rate are fixed by canopy trim. Control
authority reduces to three primitive actions:

| Action | Effect |
|---|---|
| Pull right brake | Turn right |
| Pull left brake | Turn left |
| Pull both | "Flare" — slow forward speed, steepen descent briefly |

The job of the controller is to **point the nose in the right direction while
gravity does the rest**.

---

## State estimator

Each tick the module ingests a `GuidanceInput` (see `guidance.hpp`) and derives:

### Position and altitude AGL
- Position straight from GPS (`lat_e7`, `lon_e7`).
- `alt_agl_m = gps_alt_cm / 100 − kTargetGroundAltM`. No barometer fusion
  yet — GPS altitude is noisy (±5 m typical), but mode-machine thresholds
  are spaced widely enough (70+ m gaps) that the jitter doesn't cause mode
  chatter.

### Bearing to target
Flat-earth approximation:
```
dlat_deg = (target_lat − current_lat) × 1e-7
dlon_deg = (target_lon − current_lon) × 1e-7
dy = dlat_deg × 111320 m/deg                       (north +, south −)
dx = dlon_deg × 111320 m/deg × cos(current_lat)    (east  +, west  −)
bearing = atan2(dx, dy)                            (CW from north, [-π, π])
```
Good to ~0.1% inside a kilometer — well within target-landing accuracy.

### Heading (canopy nose direction)
Two-source with a speed-based switch:

1. **GPS course-over-ground** when ground speed > `kMinSpeedForGpsCourseMps`
   (default 2 m/s). This is effectively measuring direction-of-travel over a
   long baseline and is the most reliable heading source whenever the
   parafoil is moving meaningfully.
2. **Magnetometer fallback** when slow or stationary:
   `heading = atan2(my, mx) + kImuYawOffsetCdeg + kMagDeclinationCdeg`,
   wrapped to `[-π, π]`. The fallback assumes the payload is near-level;
   tilt compensation using accelerometer data is a future improvement.

---

## Mode machine

Five states gated by altitude AGL, evaluated every tick:

| Mode | Entry condition | Servo behavior |
|---|---|---|
| `STANDBY` | No GPS fix **or** AGL > `kAltStartGuidanceM` (250 m) | Both servos at neutral (rest pulse). |
| `CRUISE` | `kAltFinalM` < AGL ≤ `kAltStartGuidanceM` (80–250 m) | Bearing-tracker active, steering toward target. |
| `FINAL` | `kAltFlareM` < AGL ≤ `kAltFinalM` (8–80 m) | Same controller as CRUISE. Thresholds reserved for a future wind-aware final-approach specialization. |
| `FLARE` | `kAltLandedM` < AGL ≤ `kAltFlareM` (1–8 m) | Both servos pulled to `kFlareAmount` (default 1.0). |
| `LANDED` | AGL ≤ `kAltLandedM` (1 m) | Both servos released. Mission over. |

The `STANDBY` gate at 250 m AGL is intentional: there's no point burning
servo duty cycle high up where positional error is small relative to
remaining glide range. Steering engages once the payload has descended into
a window where it actually has to commit to a heading.

---

## Control law (CRUISE / FINAL)

The controller is **stateless**: each tick reads aircraft state, computes a
fresh command, and emits it. No integration of past commands, no I-term, no
internal model of the servo position.

```
yaw_error = wrap_pi(bearing_to_target − current_heading)      // rad
yaw_rate  = gyro_z (after deg/s ÷ 10, then × π/180)           // rad/s

delta     = Kp · yaw_error − Kd · yaw_rate                    // dimensionless
delta     = clamp(delta, ±kBrakeMaxFraction)

if delta ≥ 0:
    right_pull = delta;    left_pull = 0
else:
    right_pull = 0;        left_pull = −delta
```

| Term | Role |
|---|---|
| `Kp · yaw_error` | Proportional: big heading error → big brake on the side that turns the canopy toward target. |
| `−Kd · yaw_rate` | Rate damping: if the canopy is already yawing toward target, subtract from the command to avoid overshoot and to suppress flip-flop on noisy GPS-course updates. |
| No I-term | Parafoils have a steady-state heading bias from wind crab; an integrator would wind up and oscillate. |

### Why stateless?

The controller does **not** track the previously-commanded servo position
or assume an internal servo state. This means:
- No drift accumulation. A noisy heading sample produces a one-tick command
  excursion, then the next tick recomputes from scratch.
- The servo's actual physical position never enters the math. The chain is
  `aircraft state → desired heading change → servo command`, with the
  closed loop running through the aircraft itself.
- Per-cycle slew limiting on the servo command is **not** implemented. If
  drop tests show the servos chattering or whining, add one in
  `Guidance::tick` — the existing `Kd · yaw_rate` damping handles the
  common case.

### Default gains

| Constant | Default | Notes |
|---|---|---|
| `kKpHeading` | 0.6 rad⁻¹ | A 90° error → ~94% brake (post-clamp). |
| `kKdYawRate` | 0.3 s | Roughly critical damping at 1 rad/s yaw rate. |
| `kBrakeMaxFraction` | 0.8 | Caps the commanded fraction; the servo cap (`kMaxAngleDeg`) provides a second, independent ceiling. |

---

## Servo command mapping

The pull fraction in `[0, 1]` is mapped to a PWM pulse width by
`ServoPair::pull_to_us`:

```
pulse_us = kRestUs + pull · (kFullPullUs − kRestUs)
```

`kFullPullUs` is derived at compile time from `kMaxAngleDeg` (default 90°),
so the PWM output **cannot** ask for more than the configured travel limit
regardless of upstream gains. This is a hardware-protection layer — even a
broken controller asking for `pull = 5.0` produces only `kMaxAngleDeg` of
rotation.

---

## Configuration

All tunable constants live in [`guidance_config.hpp`](../../../include/cansat/guidance_config.hpp).

### Per-flight values

| Constant | Default | Purpose |
|---|---|---|
| `kTargetLatE7`, `kTargetLonE7` | Tashkent placeholder | Landing point (scaled int32, × 1e7) |
| `kTargetGroundAltM` | 450 | MSL altitude at the landing spot |
| `kImuYawOffsetCdeg` | 0 | IMU mounting yaw relative to body forward |
| `kMagDeclinationCdeg` | 0 | Magnetic declination at launch site |
| `kMagOffsetX/Y/Z_uT10` | 0 | Hard-iron biases (calibrate per build) |

Per-build override via `platformio.ini`:
```ini
build_flags =
    -D GUIDANCE_TARGET_LAT_E7=413111000
    -D GUIDANCE_TARGET_LON_E7=692797000
    -D GUIDANCE_TARGET_GROUND_ALT_M=450
```

### Controller tuning

| Constant | Default | Notes |
|---|---|---|
| `kKpHeading` | 0.6 | Increase for crisper turns, decrease if oscillating |
| `kKdYawRate` | 0.3 | Increase to damp servo chatter; too much makes turns sluggish |
| `kBrakeMaxFraction` | 0.8 | Cap on commanded brake fraction |
| `kFlareAmount` | 1.0 | Symmetric pull amount during FLARE mode |
| `kMinSpeedForGpsCourseMps` | 2.0 | Speed threshold for GPS course vs mag fallback |

### Phase boundaries

| Constant | Default | Purpose |
|---|---|---|
| `kAltStartGuidanceM` | 250 m | STANDBY → CRUISE boundary |
| `kAltFinalM` | 80 m | CRUISE → FINAL boundary |
| `kAltFlareM` | 8 m | FINAL → FLARE boundary |
| `kAltLandedM` | 1 m | FLARE → LANDED boundary |

---

## Coordinate conventions

- All angles internally in **radians**.
- Headings and bearings are **CW from true north**, wrapped to `[-π, π]`.
- GPS course (`gps_course_cdeg`) is centidegrees on the wire, converted on
  ingestion.
- IMU mounting offsets are in **centidegrees** (deg × 100), signed.
- Lat/lon as scaled int32 (× 1e7), matching the rest of the GPS plumbing.

---

## Limits and caveats

1. **No wind compensation.** CRUISE flies the direct bearing to target.
   In light wind that's fine; in 5+ m/s wind you'll over/undershoot. A
   future wind-aware IP approach (loiter upwind, final leg into wind) can
   replace the bearing target during CRUISE without touching the controller.
2. **GPS course unreliable at low speed.** The 2 m/s threshold helps but
   the magnetometer fallback is not tilt-compensated. Expect rough steering
   during the first few seconds after canopy opening.
3. **Servo authority is bounded.** `kBrakeMaxFraction = 0.8` and
   `kMaxAngleDeg = 90°` together cap the turn-rate envelope. If a drop test
   shows under-actuation, increase `kMaxAngleDeg` rather than removing the
   `kBrakeMaxFraction` cap.
4. **Altitude is GPS-only.** No baro fusion. Mode thresholds are spaced to
   accommodate the noise; tighter phase control needs barometric altitude.
5. **No per-cycle slew limit on servo output.** Default behavior is to
   command a fresh absolute position every tick. Add a rate limiter only
   if drop tests show chatter.

---

## Pre-flight calibration checklist

Before trusting the controller:

1. Set `kTargetLatE7` / `kTargetLonE7` / `kTargetGroundAltM` to the actual
   landing spot.
2. Calibrate the magnetometer: rotate the assembled payload through all 3D
   orientations, record per-axis min/max of `mx_ut10` / `my_ut10` /
   `mz_ut10`, set `kMagOffsetX/Y/Z_uT10 = (max + min) / 2`.
3. Measure IMU mounting yaw vs body forward; set `kImuYawOffsetCdeg`.
4. Look up magnetic declination at the launch site
   ([NOAA calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml));
   set `kMagDeclinationCdeg`.
5. Bench-test servo direction via the 10-second self-test in
   `FlightApp::run_servo_selftest`. Verify each side spools the line in the
   direction that *shortens* it. If reversed, swap `kRestUs` / `kFullPullUs`
   in `servo_pair.hpp`.
6. Measure actual wheel rotation when commanding
   `servos_.set_left_angle_deg(45.0f)`. If it's ~30° instead of ~45°, the
   servo is a 270°-range variant — change `kUsPerDegX100` in
   `servo_pair.hpp` (~370 instead of 556).
7. At least one drop test before tuning `kKpHeading` / `kKdYawRate`. The
   defaults are conservative starting points, not final values.

---

## File map

| File | Role |
|---|---|
| [`../../../include/cansat/guidance/guidance.hpp`](../../../include/cansat/guidance/guidance.hpp) | Public interface: `GuidanceInput`, `GuidanceOutput`, `GuidanceMode`, `Guidance::tick`. |
| [`../../../include/cansat/guidance_config.hpp`](../../../include/cansat/guidance_config.hpp) | All tunable constants. |
| [`guidance.cpp`](guidance.cpp) | Implementation: mode machine + control law + heading estimator + bearing math. |
| [`../../../include/cansat/actuators/servo_pair.hpp`](../../../include/cansat/actuators/servo_pair.hpp) | Servo driver interface with compile-time angle cap. |
| [`../actuators/servo_pair.cpp`](../actuators/servo_pair.cpp) | TIM1_CH2/CH3 PWM driver for PE11 / PE13. |
| [`../../../src/flight/flight_app.cpp`](../../flight/flight_app.cpp) | Integration: calls `Guidance::tick` every 200 ms and pushes its output to `ServoPair`. |
