"""Live serial capture, controller comparison, and open-loop characterization."""
import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import statistics
import time

# Edit here or use --port/--baud. Importing this module does not open serial.
PORT = "COM3"
BAUD = 115200
OUTPUT_DIR = Path(__file__).resolve().parent / "fan_results"
FIELDS = ["time_ms", "target_rpm", "actual_rpm", "pwm_duty", "control_mode"]
MODES = ("OPEN_LOOP", "P", "PD", "PID")
COLORS = {"OPEN_LOOP": "tab:blue", "P": "tab:orange", "PD": "tab:green",
          "PID": "tab:red", "CALIBRATION": "tab:purple", "LEGACY": "tab:gray"}


@dataclass(frozen=True)
class Sample:
    time_ms: int
    target_rpm: float
    actual_rpm: float
    pwm_duty: float
    control_mode: str

    def row(self):
        return [self.time_ms, self.target_rpm, self.actual_rpm,
                self.pwm_duty, self.control_mode]


def parse_sample(line):
    """Also accept the existing four-column recordings."""
    values = line.strip().split(",")
    if len(values) not in (4, 5):
        return None
    try:
        timestamp = int(values[0])
        target, actual, duty = map(float, values[1:4])
    except ValueError:
        return None
    mode = values[4].strip().upper() if len(values) == 5 else "LEGACY"
    if (timestamp < 0 or not all(map(math.isfinite, (target, actual, duty)))
            or target < 0 or actual < 0 or not 0 <= duty <= 100
            or mode not in (*MODES, "CALIBRATION", "DONE", "LEGACY")):
        return None
    return Sample(timestamp, target, actual, duty, mode)


def load_samples(path):
    with Path(path).open(encoding="utf-8-sig") as source:
        return [sample for line in source if (sample := parse_sample(line)) is not None]


def mode_runs(samples):
    """Keep repeated modes and board resets separate."""
    runs = []
    new_run = True
    for sample in samples:
        if sample.control_mode == "DONE":
            new_run = True
            continue
        if (new_run or not runs or sample.control_mode != runs[-1][-1].control_mode
                or sample.time_ms <= runs[-1][-1].time_ms):
            runs.append([])
        runs[-1].append(sample)
        new_run = False
    return runs


def target_steps(run):
    steps = []
    for sample in run:
        if not steps or sample.target_rpm != steps[-1][-1].target_rpm:
            steps.append([])
        steps[-1].append(sample)
    return steps


def comparison_metrics(samples):
    """Per positive target step; settling requires >=1 second continuously in band."""
    results = []
    for run_index, run in enumerate(mode_runs(samples), 1):
        if run[0].control_mode not in MODES:
            continue
        for step_index, step in enumerate(target_steps(run), 1):
            target = step[0].target_rpm
            if target <= 0:
                continue
            start = step[0].time_ms
            band = max(50.0, target * 0.02)
            last_outside = max((i for i, s in enumerate(step)
                                if abs(s.actual_rpm - target) > band), default=-1)
            settled = last_outside + 1
            settling = None
            if settled < len(step) and step[-1].time_ms - step[settled].time_ms >= 1000:
                settling = (step[settled].time_ms - start) / 1000
            tail_start = start + 0.8 * (step[-1].time_ms - start)
            tail = [s.actual_rpm for s in step if s.time_ms >= tail_start]
            initial = step[0].actual_rpm
            rise = None
            if target > initial:
                low = initial + 0.1 * (target - initial)
                high = initial + 0.9 * (target - initial)
                low_time = next((s.time_ms for s in step if s.actual_rpm >= low), None)
                high_time = next((s.time_ms for s in step if s.actual_rpm >= high), None)
                if low_time is not None and high_time is not None:
                    rise = (high_time - low_time) / 1000
            excursion = (max(s.actual_rpm for s in step) - target if target >= initial
                         else target - min(s.actual_rpm for s in step))
            results.append({
                "run": run_index, "control_mode": run[0].control_mode, "step": step_index,
                "target_rpm": target, "start_ms": start,
                "steady_error_rpm": target - statistics.mean(tail),
                "rmse_rpm": math.sqrt(statistics.mean((target - s.actual_rpm) ** 2 for s in step)),
                "overshoot_percent": max(0.0, excursion) / target * 100,
                "rise_10_90_seconds": rise, "settling_seconds": settling,
            })
    return results


def characterize(samples, settle_seconds=3.0):
    """Use only constant-duty open-loop segments after their settling delay."""
    segments = []
    for run in mode_runs(samples):
        if run[0].control_mode not in ("OPEN_LOOP", "CALIBRATION", "LEGACY"):
            continue
        current = []
        for sample in run:
            if current and abs(sample.pwm_duty - current[-1].pwm_duty) > 0.05:
                segments.append(current)
                current = []
            current.append(sample)
        if current:
            segments.append(current)
    by_duty = {}
    for segment in segments:
        cutoff = segment[0].time_ms + settle_seconds * 1000
        steady = [s.actual_rpm for s in segment if s.time_ms >= cutoff]
        if len(steady) >= 5:
            by_duty.setdefault(round(segment[0].pwm_duty, 1), []).extend(steady)
    return [{"pwm_duty": duty, "mean_rpm": statistics.mean(values),
             "std_rpm": statistics.stdev(values) if len(values) > 1 else 0,
             "samples": len(values)} for duty, values in sorted(by_duty.items())]


def initial_gains(points, target_rpm):
    if len(points) < 2:
        raise ValueError("Need two settled duty levels; record TEST_CHARACTERIZE.")
    if any(b["mean_rpm"] <= a["mean_rpm"] for a, b in zip(points, points[1:])):
        raise ValueError("Measured RPM is not strictly increasing with duty; inspect the sweep.")
    pair = (points[-2], points[-1])
    for lower, upper in zip(points, points[1:]):
        if target_rpm <= upper["mean_rpm"]:
            pair = (lower, upper)
            break
    lower, upper = pair
    slope = (upper["mean_rpm"] - lower["mean_rpm"]) / (upper["pwm_duty"] - lower["pwm_duty"])
    kp = 0.5 / slope
    return {"plant_rpm_per_percent": slope, "kp": kp, "ki": kp / 2.0, "kd": kp * 0.05}


def write_dict_rows(path, rows):
    if rows:
        with Path(path).open("w", newline="", encoding="utf-8") as destination:
            writer = csv.DictWriter(destination, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def update_timeline(axes, samples):
    speed_axis, duty_axis = axes
    speed_axis.clear()
    duty_axis.clear()
    seen = set()
    for run in mode_runs(samples):
        mode = run[0].control_mode
        seconds = [s.time_ms / 1000 for s in run]
        speed_axis.plot(seconds, [s.actual_rpm for s in run], color=COLORS[mode],
                        label=mode if mode not in seen else None)
        speed_axis.plot(seconds, [s.target_rpm for s in run], "k--", alpha=0.6,
                        label="Target" if not seen else None)
        duty_axis.plot(seconds, [s.pwm_duty for s in run], color=COLORS[mode])
        seen.add(mode)
    speed_axis.set(ylabel="Speed (RPM)", title="STM32 fan control: live comparison")
    duty_axis.set(xlabel="Experiment time (s)", ylabel="PWM duty (%)", ylim=(-3, 103))
    for axis in axes:
        axis.grid(True, alpha=0.3)
    if seen:
        speed_axis.legend(loc="upper left", ncol=5)


def create_comparison(samples):
    import matplotlib.pyplot as plt
    comparison, axis = plt.subplots(figsize=(10, 6))
    target_drawn = False
    for run in mode_runs(samples):
        mode = run[0].control_mode
        if mode not in MODES:
            continue
        # Align the first positive step; the baseline appears at negative time.
        first_step = next((s.time_ms for s in run if s.target_rpm > 0), None)
        if first_step is None:
            continue
        relative = [(s.time_ms - first_step) / 1000 for s in run]
        axis.plot(relative, [s.actual_rpm for s in run], label=mode, color=COLORS[mode])
        axis.plot(relative, [s.target_rpm for s in run], "k--", alpha=0.4,
                  label="Target sequence" if not target_drawn else None)
        target_drawn = True
    axis.set(xlabel="Time since first target step (s)", ylabel="Speed (RPM)",
             title="Open-loop / P / PD / PID: aligned step responses")
    axis.grid(True, alpha=0.3)
    if target_drawn:
        axis.legend()
    comparison.tight_layout()
    return comparison


def save_plots(samples, base):
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    update_timeline(axes, samples)
    figure.tight_layout()
    figure.savefig(base.with_name(base.name + "_response.png"), dpi=180)
    plt.close(figure)
    comparison = create_comparison(samples)
    comparison.savefig(base.with_name(base.name + "_comparison.png"), dpi=180)
    plt.close(comparison)
    write_dict_rows(base.with_name(base.name + "_metrics.csv"), comparison_metrics(samples))


def save_characterization(samples, base, settle_seconds, target):
    import matplotlib.pyplot as plt
    points = characterize(samples, settle_seconds)
    write_dict_rows(base.with_name(base.name + "_calibration.csv"), points)
    gains = initial_gains(points, target)
    figure, axis = plt.subplots(figsize=(8, 5))
    axis.errorbar([p["pwm_duty"] for p in points], [p["mean_rpm"] for p in points],
                  yerr=[p["std_rpm"] for p in points], fmt="o-", capsize=4)
    axis.set(xlabel="PWM duty (%)", ylabel="Settled RPM", title="Measured open-loop fan curve")
    axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(base.with_name(base.name + "_calibration.png"), dpi=180)
    plt.close(figure)
    for point in points:
        print(f"{point['pwm_duty']:5.1f}% -> {point['mean_rpm']:7.1f} RPM "
              f"(std {point['std_rpm']:.1f}, n={point['samples']})")
    print(f"Plant slope near {target:g} RPM: {gains['plant_rpm_per_percent']:.4f} RPM/percent")
    print(f"Initial Kp={gains['kp']:.6f}, Ki={gains['ki']:.6f}, Kd={gains['kd']:.6f}")
    if len(points) == 2:
        print("Only two duty levels: provisional secant estimate, not a measured local curve.")
    print("Paste these {rpm, duty_percent} points into openLoopMap in main.c:")
    for point in points:
        print(f"  {{{point['mean_rpm']:.2f}f, {point['pwm_duty']:.1f}f}},")
    return gains


def capture(args, base):
    import serial
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    samples = []
    pending = bytearray()
    figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    started = time.monotonic()
    with serial.Serial(args.port, args.baud, timeout=0) as connection, \
            base.with_suffix(".csv").open("w", newline="", encoding="utf-8") as destination:
        writer = csv.writer(destination)
        writer.writerow(FIELDS)
        print(f"Listening on {args.port} at {args.baud}. Press NUCLEO RESET to start.")
        print("Close the plot or Ctrl+C to finish; DONE also ends capture automatically.")

        def update(_frame):
            nonlocal pending
            try:
                available = connection.in_waiting
                if available:
                    pending.extend(connection.read(min(available, 8192)))
            except serial.SerialException as error:
                print(f"Serial connection ended: {error}")
                plt.close(figure)
                return []
            if len(pending) > 65536:
                pending.clear()
            while b"\n" in pending:
                raw, _, pending = pending.partition(b"\n")
                sample = parse_sample(raw.decode("ascii", errors="ignore"))
                if sample is None:
                    continue
                if sample.control_mode == "DONE" and not samples:
                    continue  # wait for RESET if the board already finished a previous run
                samples.append(sample)
                writer.writerow(sample.row())
                if sample.control_mode == "DONE":
                    plt.close(figure)
                    break
            destination.flush()
            update_timeline(axes, samples)
            figure.tight_layout()
            if args.duration and time.monotonic() - started >= args.duration:
                plt.close(figure)
            return []

        animation = FuncAnimation(figure, update, interval=100, cache_frame_data=False)
        try:
            plt.show()
        except KeyboardInterrupt:
            plt.close(figure)
        finally:
            # Matplotlib clears event_source when the figure is closed.
            if animation.event_source is not None:
                animation.event_source.stop()
            if samples:
                save_plots(samples, base)
                print(f"Comparison graph: {base.with_name(base.name + '_comparison.png')}")
            print(f"Saved {len(samples)} samples to {base.with_suffix('.csv')}")
    return samples


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--duration", type=float, default=0, help="live capture limit, seconds")
    parser.add_argument("--csv", type=Path, help="plot existing four- or five-column CSV")
    parser.add_argument("--characterize", type=Path, help="open-loop curve and initial gains")
    parser.add_argument("--target", type=float, default=3500, help="RPM for local slope calculation")
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--no-show", action="store_true", help="headless offline plotting")
    args = parser.parse_args(argv)
    if (not all(map(math.isfinite, (args.duration, args.settle_seconds, args.target)))
            or args.duration < 0 or args.settle_seconds < 0 or args.target <= 0):
        parser.error("Duration/settling time must be nonnegative and target RPM positive.")
    if args.no_show and not (args.csv or args.characterize):
        parser.error("--no-show is for offline --csv or --characterize.")
    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    source = args.characterize or args.csv
    if source:
        samples = load_samples(source)
        if not samples:
            parser.error("No valid samples in the input CSV.")
        base = args.output_dir / source.stem
        if args.characterize:
            try:
                save_characterization(samples, base, args.settle_seconds, args.target)
            except ValueError as error:
                parser.error(str(error))
        else:
            save_plots(samples, base)
        if not args.no_show:
            import matplotlib.pyplot as plt
            figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
            update_timeline(axes, samples)
            figure.tight_layout()
            if any(s.control_mode in MODES for s in samples):
                create_comparison(samples)
            plt.show()
        print(f"Plots saved under {args.output_dir.resolve()}")
        return
    base = args.output_dir / datetime.now().strftime("fan_%Y%m%d_%H%M%S_%f")
    samples = capture(args, base)
    if any(s.control_mode == "CALIBRATION" for s in samples):
        try:
            save_characterization(samples, base, args.settle_seconds, args.target)
        except ValueError as error:
            print(f"Calibration incomplete: {error}")
    elif any(s.control_mode in MODES for s in samples):
        # Serial is closed before presenting the final aligned comparison.
        import matplotlib.pyplot as plt
        create_comparison(samples)
        plt.show()


if __name__ == "__main__":
    main()

