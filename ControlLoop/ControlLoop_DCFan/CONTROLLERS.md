# Fan controller experiment

The existing TIM2 CH3 / PB10 PWM, TIM1 CH1 / PA8 tach interrupt and USART2
115200-baud transport remain in use. Peripheral initialization, wiring,
interrupt vectors and the CubeMX configuration have not been changed.

## Settings and first run

Edit the settings at the top of `Src/main.c`:

```c
volatile float targetRpm = 3500.0f;
volatile FanControlMode controllerMode = FAN_MODE_PID;
volatile FanTestPlan testPlan = TEST_COMPARE;

static const float Kp = 0.0122f;
static const float Ki = 0.0061f;
static const float Kd = 0.00061f;
static const float derivativeFilterSeconds = 0.20f;
static const bool useFeedforward = true;
static const uint32_t controlPeriodMs = 100U;
```

Refresh the CubeIDE project, build and flash. `Src/fan_controller.c` must be
included in the build; CubeIDE normally discovers it under `Src` after refresh.
Close Tera Term and any other program using the COM port. From the project
directory, start:

```powershell
python ControlLoopAnalysis/fan_monitor.py --port COM3
```

Use your actual ST-LINK COM port. Press the NUCLEO RESET button **after Python
is listening**, so the recording includes the complete first controller test.
Python keeps waiting for reset if the board is already transmitting `DONE`.

`TEST_COMPARE` runs these targets for each mode, in order OPEN_LOOP, P, PD, PID:

| Target | Duration |
| --- | --- |
| 0 RPM | 5 s |
| `targetRpm` = 3500 RPM | 10 s |
| `0.7 * targetRpm` = 2450 RPM | 10 s |
| `targetRpm` = 3500 RPM | 10 s |
| 0 RPM | 5 s |

The full comparison takes 160 seconds and finishes with PWM at zero. Each
mode uses the same target sequence and starts with reset controller state.
The final stop and next mode's baseline give the fan time to coast down.
State also resets whenever the commanded target is zero.

For one controller holding 3500 RPM continuously, use `TEST_HOLD` and choose
`controllerMode` from `FAN_MODE_OPEN_LOOP`, `FAN_MODE_P`, `FAN_MODE_PD`,
`FAN_MODE_PID`. The three volatile settings can also be changed in the
debugger. Changing `testPlan` restarts the test timeline; changing the mode
in HOLD resets integral and derivative state. Rebuild after changing gains,
map points or period, which are copied into the controller configuration at boot.
For comparable experiments, keep settings unchanged during a comparison.

Closing Python ends recording; it does not send a stop command. In HOLD,
set `targetRpm` to zero in the debugger or disconnect power to stop the fan.

## Controller implementation

`Inc/fan_controller.h` and `Src/fan_controller.c` contain the HAL-independent
controller. Duty is expressed in percent, RPM in revolutions/minute and time
in seconds. All modes saturate output to 0..100%. A zero target explicitly
commands zero PWM and clears controller state.

With the default shared feedforward enabled:

```text
e[k] = target[k] - measured[k]
u_ff = interpolated openLoopMap(target)

OPEN_LOOP: u = u_ff
P:         u = u_ff + Kp*e
PD:        u = u_ff + Kp*e - Kd*filtered(measurement rate)
PID:       u = u_ff + Kp*e + integral - Kd*filtered(measurement rate)

I_candidate = I_previous + Ki*e*Ts
alpha = Ts / (derivativeFilterSeconds + Ts)
d_filtered[k] = d_filtered[k-1] + alpha*(raw_rate - d_filtered[k-1])
```

PID uses conditional integration: it rejects an integral update that would
drive an already saturated output farther into its limit, while allowing
unwinding. The integral term also has independent bounds. Derivative action
uses measurement changes, avoiding a target-step derivative kick. Mode changes
clear the integral, derivative filter and measurement history. These techniques
follow the usual [clamping anti-windup](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)
and [discrete filtered PID](https://www.mathworks.com/help/simulink/slref/discretepidcontroller.html)
approaches. Resetting is deliberate; mode transfer is not guaranteed bumpless.

The common feedforward baseline makes P/PD/PID corrections directly comparable
to the measured open-loop estimate. Set `useFeedforward = false` for classical
feedback-only P/PD/PID; OPEN_LOOP still uses the map. P and PD can retain steady
error with model mismatch; integral action can remove that error when the target
is reachable. A PID label alone does not imply a better or stable hardware response.

## Timing and RPM freshness

The scheduler advances an absolute deadline by 100 ms each tick, so serial
transmit time does not accumulate into the sample period. Controllers use the
configured fixed `Ts = 0.1 s`. UART transmission has a 20 ms timeout. This is a
cooperative HAL tick scheduler, with millisecond scheduling jitter, not a hard
real-time interrupt task. If a deadline is late by more than 5 ms, state is reset, the
measurement baseline is refreshed, and the last output is held until a fresh
period is available. It never performs several updates using one stale sample.

The verified rising-edge pulse counter is retained. The capture ISR additionally
records the latest edge's HAL millisecond timestamp. A short interrupt-protected
snapshot reads count/time together. After the first sample, RPM uses the number
of new pulses divided by the time between the last edge of the previous window
and the last edge of the current window. This reduces the old ~300 RPM counting
steps at 100 ms, without changing timer clock settings or reusing a one-second
RPM measurement. The first window falls back to count/actual elapsed time.
No pulses in the current window gives zero RPM. Unsigned differences support
counter/tick rollover during regular sampling.

Near 3500 RPM, a tach pulse arrives about every 8.6 ms, so each control sample
normally contains about 12 pulses. Millisecond timestamp resolution still causes
measurement noise; derivative filtering reduces its effect. At very low speeds,
100 ms may be too short to receive a pulse; increase the interval or use a finer
capture-period estimator before controlling that region. Missing wiring and a
stopped rotor both produce no pulses. The controller cannot distinguish them.

## CSV and Python results

The first four fields retain their order and units. Mode is appended:

```text
time_ms,target_rpm,actual_rpm,pwm_duty,control_mode
5100,3500,3200,89.1,P
```

Firmware sends numeric data rows with a mode name, without a header or banners.
Time is elapsed experiment time; PWM is the rounded timer duty actually applied.
RPM is measured before the new PWM command is applied at that tick. Python
adds the CSV header and accepts old four-column logs as `LEGACY`.

Results are saved in `ControlLoopAnalysis/fan_results` using unique run names:

- `fan_...csv`: every received sample.
- `fan_..._response.png`: complete RPM/target and PWM timeline, colored by mode.
- `fan_..._comparison.png`: four RPM responses aligned to their first positive
  target step; the pre-step baseline appears at negative time.
- `fan_..._metrics.csv`: per-step tracking RMSE, final-window steady error,
  directional overshoot (undershoot for downward steps), 10..90% rise time for
  upward steps and settling time. Settling uses a +/-2% band with a 50 RPM floor,
  and requires at least one second continuously inside the band. Empty settling
  fields mean settling was not demonstrated during the recorded step.

Python saves results on `DONE`, window close or Ctrl+C, then displays the aligned
controller comparison in a final window. Existing CSV/PNG results
are preserved. To replot without hardware:

```powershell
python ControlLoopAnalysis/fan_monitor.py --csv ControlLoopAnalysis/fan_results/your_run.csv --no-show
```

## Characterize the fan and refine gains

The supplied old log contains only 0% and 100% duty. Discarding the first three
seconds of each recorded constant-duty segment leaves a mean **4093.65 RPM at
100%**, based on 60 running samples, and zero at 0%. Therefore the initial
endpoint/secant estimate is:

```text
Kplant = (4093.65 - 0) / (100 - 0) = 40.9365 RPM / duty-percent
Kp = 0.5 / Kplant = 0.012214 duty-percent / RPM  -> use 0.0122
Ki = Kp / 2 s      -> use 0.0061
Kd = Kp * 0.05 s   -> use 0.00061
```

The factor 0.5 is a conservative initial static-loop-gain choice, not a
stability proof or a fitted dynamic model. The 2 s integral time and 0.05 s
derivative time are starting assumptions. All require hardware validation.
The provisional map is `{0 RPM, 0%}` and `{4093.65 RPM, 100%}`; at 3500 RPM it
predicts about 85.5% duty. This does not establish a linear measured curve near
3500 RPM. Do not substitute the manufacturer's nominal maximum for your data.

To measure the curve, set `testPlan = TEST_CHARACTERIZE`, rebuild and flash.
Record with the same Python command. It applies 0, 20, 35, 50, 65, 80, 100, 0%
for eight seconds each, then stops at 64 seconds. Rows use `CALIBRATION` in the
mode column. Python automatically writes the calibration table/plot and prints
map points plus starting gains after capture. It excludes each segment's first
three seconds; inspect the data and use `--settle-seconds 5` if necessary.

For an existing recording:

```powershell
python ControlLoopAnalysis/fan_monitor.py --characterize ControlLoopAnalysis/fan_results/your_calibration.csv --target 3500 --no-show
```

Use the two settled duty levels bracketing 3500 RPM to calculate the local
`delta_RPM / delta_duty` slope. Paste the printed monotonic points into
`openLoopMap` and update the gains at the top of `main.c`, then rerun all four
controllers with the same sequence. Start by checking P, add a small filtered
D term, and introduce I gradually. Do not compare runs made with different maps
or gains without recording that change alongside the results.

## Verification

From the project directory:

```powershell
python -m unittest discover -s ControlLoopAnalysis -p "test_*.py" -v
gcc -std=c11 -Wall -Wextra -Werror -IInc Src/fan_controller.c tests/test_fan_controller.c -o .build/test_fan_controller.exe
.\.build\test_fan_controller.exe
```

Create `.build` first if it does not exist. Tests cover controller interpolation,
limits, anti-windup, derivative filtering/kick, mode resets, invalid inputs and
a reference plant simulation. Python tests cover CSV compatibility, incomplete
serial lines, completion, board resets, metrics, characterization and PNG output.
Synthetic test traces are not hardware measurements. Validate the new firmware
on the actual fan before treating the gains or performance metrics as tuned.
