import argparse
from contextlib import contextmanager
import csv
from pathlib import Path
import shutil
import uuid
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import fan_monitor as monitor


@contextmanager
def temporary_output():
    root = (Path(__file__).resolve().parent.parent / ".build" / "python-tests").resolve()
    root.mkdir(parents=True, exist_ok=True)
    directory = (root / uuid.uuid4().hex).resolve()
    # Verify the final cleanup target is inside this test's workspace directory.
    if not directory.is_relative_to(root):
        raise RuntimeError("Unexpected test cleanup path")
    directory.mkdir()
    try:
        yield directory
    finally:
        shutil.rmtree(directory)


class MonitorTests(unittest.TestCase):
    def test_parser_accepts_new_and_existing_csv(self):
        self.assertEqual(monitor.parse_sample("100,3500,3480,85.8,PID\r\n").control_mode, "PID")
        self.assertEqual(monitor.parse_sample("100,0,4000,100").control_mode, "LEGACY")
        for line in (",".join(monitor.FIELDS), "garbage", "1,2,nan,3,P", "1,2,3,101,P",
                     "-1,2,3,5,P", "1,2,3,5,UNKNOWN"):
            self.assertIsNone(monitor.parse_sample(line))

    def test_resets_and_mode_switches_are_not_joined(self):
        samples = [monitor.Sample(t, 3500, 3400, 80, mode) for t, mode in
                   ((100, "P"), (200, "P"), (300, "PID"), (100, "PID"))]
        self.assertEqual([len(run) for run in monitor.mode_runs(samples)], [2, 1, 1])

    def test_metrics_require_sustained_settling(self):
        samples = [monitor.Sample(t, 3500, 3500 if t >= 1000 else 3000, 85, "PID")
                   for t in range(0, 3100, 100)]
        metric = monitor.comparison_metrics(samples)[0]
        self.assertEqual(metric["settling_seconds"], 1.0)
        self.assertEqual(metric["steady_error_rpm"], 0)
        self.assertEqual(metric["overshoot_percent"], 0)
        samples[-1] = monitor.Sample(3000, 3500, 3700, 85, "PID")
        self.assertIsNone(monitor.comparison_metrics(samples)[0]["settling_seconds"])
        downward = [monitor.Sample(0, 2500, 3500, 60, "PID"),
                    monitor.Sample(100, 2500, 2400, 60, "PID")]
        self.assertEqual(monitor.comparison_metrics(downward)[0]["overshoot_percent"], 4)

    def test_calibration_discards_transients_and_feedback_modes(self):
        samples = []
        for index, duty in enumerate((0, 50, 100)):
            for offset in range(0, 8000, 100):
                actual = duty * 40 if offset >= 3000 else 9999
                samples.append(monitor.Sample(index * 8000 + offset, 0, actual, duty, "CALIBRATION"))
        samples.append(monitor.Sample(25000, 3500, 9999, 70, "PID"))
        points = monitor.characterize(samples)
        self.assertEqual([p["mean_rpm"] for p in points], [0, 2000, 4000])
        gains = monitor.initial_gains(points, 3500)
        self.assertAlmostEqual(gains["kp"], 0.0125)
        with self.assertRaises(ValueError):
            monitor.initial_gains(points[:1], 3500)

    def test_offline_plots_and_metrics_for_all_modes(self):
        samples = []
        for index, mode in enumerate(monitor.MODES):
            for offset in range(0, 4000, 100):
                target = 3500 if offset >= 1000 else 0
                samples.append(monitor.Sample(index * 4000 + offset, target, target, 85 if target else 0, mode))
        with temporary_output() as directory:
            base = Path(directory) / "synthetic_test"
            monitor.save_plots(samples, base)
            for suffix in ("_response.png", "_comparison.png", "_metrics.csv"):
                self.assertGreater(base.with_name(base.name + suffix).stat().st_size, 0)
            with base.with_name(base.name + "_metrics.csv").open() as source:
                self.assertEqual({r["control_mode"] for r in csv.DictReader(source)}, set(monitor.MODES))

    def test_live_capture_keeps_fragmented_lines_and_stops_on_done(self):
        class FakeSerial:
            def __init__(self, *args, **kwargs):
                self.fragments = [b"999,0,0,0,DONE\n100,3500,34", b"80,85.8,PID\r\n200,3500,3500,85.8,PID\n",
                                  b"300,0,3400,0,DONE\n"]
            def __enter__(self): return self
            def __exit__(self, *_): pass
            @property
            def in_waiting(self): return len(self.fragments[0]) if self.fragments else 0
            def read(self, _size): return self.fragments.pop(0)

        class FakeAnimation:
            event_source = None
            def __init__(self, _figure, callback, **_kwargs):
                self.callback = callback
                callbacks.append(callback)

        callbacks = []
        with temporary_output() as directory:
            args = argparse.Namespace(port="TEST", baud=115200, duration=0)
            base = Path(directory) / "capture"
            with patch("serial.Serial", FakeSerial), \
                    patch("matplotlib.animation.FuncAnimation", FakeAnimation), \
                    patch.object(plt, "show", side_effect=lambda: [callbacks[0](i) for i in range(3)]):
                samples = monitor.capture(args, base)
            self.assertEqual(len(samples), 3)
            self.assertEqual(samples[0].actual_rpm, 3480)
            self.assertEqual(samples[-1].control_mode, "DONE")
            self.assertEqual(len(monitor.load_samples(base.with_suffix(".csv"))), 3)


if __name__ == "__main__":
    unittest.main()
