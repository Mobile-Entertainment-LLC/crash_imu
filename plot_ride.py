"""
plot_ride.py - Plot accel_mag at 20Hz, 50Hz, 100Hz from ride CSV.
Saves output as ride_plot.png in the same folder as the CSV.

Run:  python3 plot_ride.py ride.csv
"""

import sys
import os
import csv
import math
import matplotlib
matplotlib.use("Agg")  # no display needed, saves to file
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def load_csv(path):
    times = []
    accel = []
    verdicts = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                times.append(float(row["time"]))
                accel.append(float(row["accel_mag"]))
                verdicts.append(row.get("verdict", "").strip())
            except (ValueError, KeyError):
                continue
    if not times:
        raise SystemExit("No data found in CSV. Check column names.")
    t0 = times[0]
    times = [t - t0 for t in times]
    return times, accel, verdicts


def downsample(times, accel, verdicts, target_hz, source_hz):
    step = max(1, round(source_hz / target_hz))
    return times[::step], accel[::step], verdicts[::step]


def find_events(times, verdicts):
    """Return list of (time, label) for non-empty verdicts."""
    events = []
    for t, v in zip(times, verdicts):
        if v and v != "NORMAL" and v != "":
            events.append((t, v))
    return events


def plot_series(ax, times, accel, events, hz_label, crash_threshold=2.0):
    ax.plot(times, accel, color="#1D9E75", linewidth=0.9, label="accel_mag (g)")

    # highlight spikes above threshold in red
    for i in range(len(accel)):
        if accel[i] > crash_threshold:
            ax.axvline(times[i], color="#E24B4A", alpha=0.15, linewidth=0.8)

    # mark verdict events with a vertical dashed line + label
    labeled = set()
    for t, label in events:
        color = {
            "CRASH":     "#E24B4A",
            "FALL":      "#D85A30",
            "HARD_STOP": "#BA7517",
            "NORMAL":    "#888780",
        }.get(label, "#888780")

        ax.axvline(t, color=color, linewidth=1.2, linestyle="--")
        key = f"{label}_{round(t, 1)}"
        if key not in labeled:
            ax.text(t + 0.1, ax.get_ylim()[1] * 0.92 if ax.get_ylim()[1] > 1 else 4.5,
                    f"{label}\n{t:.1f}s",
                    fontsize=7, color=color, va="top")
            labeled.add(key)

    ax.set_ylabel("g", fontsize=9)
    ax.set_title(hz_label, fontsize=10, fontweight="normal", loc="left", pad=4)
    ax.set_ylim(bottom=0)
    ax.tick_params(axis="both", labelsize=8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", linewidth=0.4, alpha=0.5)


def main():
    if len(sys.argv) < 2:
        raise SystemExit("Usage: python3 plot_ride.py ride.csv")

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        raise SystemExit(f"File not found: {csv_path}")

    print(f"Loading {csv_path}...")
    times, accel, verdicts = load_csv(csv_path)

    duration = times[-1] - times[0]
    source_hz = round(len(times) / duration) if duration > 0 else 100
    print(f"  {len(times)} samples, {duration:.1f}s, ~{source_hz} Hz")

    events = find_events(times, verdicts)
    if events:
        print(f"  Events found: {events}")
    else:
        print("  No verdict events found (all NORMAL or empty).")

    t20,  a20,  _  = downsample(times, accel, verdicts, 20,  source_hz)
    t50,  a50,  _  = downsample(times, accel, verdicts, 50,  source_hz)
    t100, a100, _  = times, accel, verdicts

    # always use full-resolution events so 20Hz/50Hz graphs don't miss verdicts
    all_events = events

    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    fig.suptitle("IMU accel magnitude — crash detection", fontsize=12, y=0.98)

    plot_series(axes[0], t20,  a20,  all_events, "20 Hz")
    plot_series(axes[1], t50,  a50,  all_events, "50 Hz")
    plot_series(axes[2], t100, a100, all_events, "100 Hz")

    axes[2].set_xlabel("time (s)", fontsize=9)

    # legend
    patches = [
        mpatches.Patch(color="#1D9E75", label="accel_mag"),
        mpatches.Patch(color="#E24B4A", label="CRASH"),
        mpatches.Patch(color="#D85A30", label="FALL"),
        mpatches.Patch(color="#BA7517", label="HARD_STOP"),
    ]
    fig.legend(handles=patches, loc="lower center", ncol=4,
               fontsize=8, frameon=False, bbox_to_anchor=(0.5, 0.01))

    plt.tight_layout(rect=[0, 0.04, 1, 0.97])

    out_path = os.path.splitext(csv_path)[0] + "_plot.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()