#!/usr/bin/env python3
# Copyright 2026 masayuki-kono
# SPDX-License-Identifier: MIT
"""Plot the CSV produced by the examples/tracking_capture_* programs.

Usage:
    plot_tracking.py velocity velocity_tracking.csv [out.png]
    plot_tracking.py position position_tracking.csv [out.png]
    plot_tracking.py pp pp_tracking.csv [out.png]
    plot_tracking.py current current_tracking.csv [out.png]
    plot_tracking.py operation operation_tracking.csv [out.png]

Prints per-plateau steady-state statistics and writes the tracking plot
(default output: <csv basename>.png). Requires matplotlib.
"""

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

MODES = {
    # mode: (tracked column, unit, settle_skip [s], plot title)
    "velocity": ("velocity", "rad/s", 1.5, "Velocity mode: command tracking"),
    "position": ("position", "rad", 2.0, "CSP position mode: command tracking"),
    "pp": ("position", "rad", 2.0, "PP position mode: command tracking"),
    "current": ("iq", "A", 0.5, "Current mode: command tracking"),
    "operation": ("position", "rad", 1.5, "Operation control: command tracking"),
}


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {key: [float(row[key]) for row in rows] for key in rows[0]}


def plateau_stats(times, targets, actuals, settle_skip):
    """Yields (t_start, t_end, target, mean, max_abs_error) per plateau,
    excluding the first `settle_skip` seconds of each plateau."""
    seg_start = 0
    for i in range(1, len(targets) + 1):
        if i < len(targets) and targets[i] == targets[seg_start]:
            continue
        t0 = times[seg_start]
        idx = [j for j in range(seg_start, i) if times[j] >= t0 + settle_skip]
        if idx:
            target = targets[seg_start]
            values = [actuals[j] for j in idx]
            yield (
                t0,
                times[i - 1],
                target,
                sum(values) / len(values),
                max(abs(v - target) for v in values),
            )
        seg_start = i


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in MODES:
        print(__doc__, file=sys.stderr)
        return 1
    mode = sys.argv[1]
    csv_path = Path(sys.argv[2])
    out_path = Path(sys.argv[3]) if len(sys.argv) > 3 else csv_path.with_suffix(".png")

    tracked, unit, settle_skip, title = MODES[mode]
    data = load(csv_path)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)

    if mode == "velocity":
        axes[1].plot(data["t"], data["position"], color="tab:green", linewidth=1.2)
        axes[1].set_ylabel("position [rad]")
        axes[1].set_title("Resulting position (unwrapped, continuous)")
    elif mode == "current":
        axes[1].plot(data["t"], data["velocity"], color="tab:purple", linewidth=1.0)
        axes[1].set_ylabel("velocity [rad/s]")
        axes[1].set_title("Resulting velocity (unloaded shaft)")
    else:
        axes[1].plot(data["t"], data["velocity"], color="tab:purple", linewidth=1.0)
        axes[1].set_ylabel("velocity [rad/s]")
        axes[1].set_title("Velocity during the moves")

    axes[0].set_title(title)
    axes[0].step(
        data["t"],
        data["target"],
        where="post",
        color="tab:red",
        linewidth=1.8,
        label=f"target {tracked}",
    )
    axes[0].plot(
        data["t"],
        data[tracked],
        color="tab:blue",
        linewidth=1.0,
        label=f"actual {tracked}",
    )
    axes[0].set_ylabel(f"{tracked} [{unit}]")
    axes[0].legend(loc="upper right")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    axes[1].set_xlabel("time [s]")

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    print(f"plot saved -> {out_path}")

    print(f"steady-state per plateau (first {settle_skip:g} s excluded):")
    for t0, t1, target, mean, max_err in plateau_stats(
        data["t"], data["target"], data[tracked], settle_skip
    ):
        print(
            f"  {t0:5.1f}-{t1:5.1f}s target {target:+.3f} {unit}: "
            f"mean {mean:+.4f}, max|err| {max_err:.4f}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
