"""
Robot Follower — Serial Analysis Tool
======================================
Usage:
    python analyze.py --port COM3 --test step
    python analyze.py --port /dev/ttyACM0 --test ramp
    python analyze.py --csv saved_run.csv          # replay from file

Arguments:
    --port   Serial port of the Pico
    --baud   Baud rate (default 115200)
    --test   Test name tag for saved files (default: test)
    --csv    Load existing CSV instead of live serial
    --dur    Duration to record in seconds (default: 30)

Output:
    data/<test>_<timestamp>.csv     raw CSV
    plots/<test>_<timestamp>.png    all metrics in one figure
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime
from collections import deque

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── optional: live serial ──────────────────────────────────────────
try:
    import serial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

# ================================================================
#  CONFIG
# ================================================================
DESIRED_CM   = 25.0
COLS = ["time_ms", "dist_cm", "error_cm", "v_base",
        "pwm_L", "pwm_R", "drift", "rms_error", "iae", "overshoot_pct"]

os.makedirs("data",  exist_ok=True)
os.makedirs("plots", exist_ok=True)

# ================================================================
#  READING
# ================================================================
def read_serial(port, baud, duration_s, out_path):
    if not SERIAL_OK:
        print("pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    rows = []
    print(f"Opening {port} at {baud} baud. Recording for {duration_s}s ...")

    with serial.Serial(port, baud, timeout=1) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()

        deadline = time.time() + duration_s
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(COLS)

            header_skipped = False
            while time.time() < deadline:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                # skip the header the Pico prints at startup
                if "time_ms" in line:
                    header_skipped = True
                    continue
                parts = line.split(",")
                if len(parts) != len(COLS):
                    continue
                try:
                    row = [float(p) for p in parts]
                    rows.append(row)
                    writer.writerow(row)
                    # live feedback
                    elapsed = row[0] / 1000
                    print(f"\r  t={elapsed:6.1f}s  dist={row[1]:5.1f}cm  "
                          f"err={row[2]:+5.1f}  rms={row[7]:4.2f}  "
                          f"iae={row[8]:6.2f}", end="", flush=True)
                except ValueError:
                    continue

    print(f"\nSaved {len(rows)} rows → {out_path}")
    return np.array(rows)


def read_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append([float(row[c]) for c in COLS])
            except (ValueError, KeyError):
                continue
    print(f"Loaded {len(rows)} rows from {path}")
    return np.array(rows)


# ================================================================
#  METRICS COMPUTATION
# ================================================================
def compute_metrics(data):
    t   = data[:, 0] / 1000.0   # seconds
    d   = data[:, 1]             # dist_cm
    e   = data[:, 2]             # error_cm
    vb  = data[:, 3]             # v_base
    pL  = data[:, 4]             # pwm_L
    pR  = data[:, 5]             # pwm_R
    dr  = data[:, 6]             # drift

    # ── steady-state error (last 20% of run) ──
    tail = int(len(e) * 0.8)
    sse  = np.mean(e[tail:])

    # ── overshoot ──
    peak = np.max(d)
    overshoot_pct = max(0, (peak - DESIRED_CM) / DESIRED_CM * 100)

    # ── undershoot (robot going too close) ──
    trough = np.min(d)
    undershoot_pct = max(0, (DESIRED_CM - trough) / DESIRED_CM * 100)

    # ── rise time: first time error crosses from >5cm to within 10% of target ──
    target_band = DESIRED_CM * 0.10
    in_band = np.where(np.abs(e) < target_band)[0]
    rise_time = t[in_band[0]] - t[0] if len(in_band) > 0 else None

    # ── settling time: last time error goes outside 5% band ──
    band_5pct = DESIRED_CM * 0.05
    outside = np.where(np.abs(e) > band_5pct)[0]
    settle_time = t[outside[-1]] - t[0] if len(outside) > 0 else 0.0

    # ── RMS error (whole run) ──
    rms = np.sqrt(np.mean(e**2))

    # ── IAE (whole run) ──
    dt_arr = np.diff(t, prepend=t[0])
    iae = np.sum(np.abs(e) * dt_arr)

    # ── ITAE (integral of time × absolute error — punishes late errors more) ──
    itae = np.sum(t * np.abs(e) * dt_arr)

    return dict(
        t=t, dist=d, error=e, v_base=vb, pwm_L=pL, pwm_R=pR, drift=dr,
        steady_state_error=sse,
        overshoot_pct=overshoot_pct,
        undershoot_pct=undershoot_pct,
        rise_time=rise_time,
        settling_time=settle_time,
        rms=rms,
        iae=iae,
        itae=itae,
        peak_dist=peak,
        trough_dist=trough,
    )


# ================================================================
#  PLOTTING
# ================================================================
def plot_all(m, test_name, save_path):
    matplotlib.rcParams.update({
        "font.size": 10,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "legend.fontsize": 8,
    })

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f"Robot follower — {test_name}", fontsize=12, fontweight="bold")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

    t = m["t"]

    # ── 1. Distance vs time ──────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(t, m["dist"],  color="#2563eb", linewidth=1.2, label="Measured dist")
    ax1.axhline(DESIRED_CM, color="#dc2626", linewidth=1, linestyle="--", label="Setpoint 25cm")
    ax1.fill_between(t, DESIRED_CM*0.95, DESIRED_CM*1.05,
                     alpha=0.12, color="#16a34a", label="±5% band")
    if m["rise_time"] is not None:
        ax1.axvline(m["rise_time"], color="#d97706", linewidth=1,
                    linestyle=":", label=f"Rise time {m['rise_time']:.2f}s")
    ax1.axvline(m["settling_time"], color="#7c3aed", linewidth=1,
                linestyle=":", label=f"Settle {m['settling_time']:.2f}s")
    ax1.set_ylabel("Distance (cm)")
    ax1.set_xlabel("Time (s)")
    ax1.set_title("Distance response")
    ax1.legend(loc="upper right", framealpha=0.7)
    ax1.grid(True, alpha=0.3)

    # ── 2. Error vs time ─────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, :2])
    ax2.plot(t, m["error"], color="#dc2626", linewidth=1, label="Error")
    ax2.axhline(0, color="black", linewidth=0.8)
    ax2.fill_between(t, -1.25, 1.25, alpha=0.1, color="#16a34a", label="±1.25cm")
    ax2.set_ylabel("Error (cm)")
    ax2.set_xlabel("Time (s)")
    ax2.set_title("Tracking error")
    ax2.legend(loc="upper right", framealpha=0.7)
    ax2.grid(True, alpha=0.3)

    # ── 3. PWM outputs ───────────────────────────────────────────
    ax3 = fig.add_subplot(gs[2, :2])
    ax3.plot(t, m["pwm_L"], color="#2563eb", linewidth=0.9, label="pwm_L", alpha=0.8)
    ax3.plot(t, m["pwm_R"], color="#16a34a", linewidth=0.9, label="pwm_R", alpha=0.8)
    ax3.plot(t, m["v_base"], color="#d97706", linewidth=1.1,
             linestyle="--", label="v_base", alpha=0.9)
    ax3.axhline(0, color="black", linewidth=0.6)
    ax3.set_ylabel("PWM / v_base")
    ax3.set_xlabel("Time (s)")
    ax3.set_title("Motor commands")
    ax3.legend(loc="upper right", framealpha=0.7)
    ax3.grid(True, alpha=0.3)

    # ── 4. Encoder drift ─────────────────────────────────────────
    ax4 = fig.add_subplot(gs[0, 2])
    ax4.plot(t, m["drift"], color="#7c3aed", linewidth=0.9)
    ax4.axhline(0, color="black", linewidth=0.6)
    ax4.set_ylabel("Cumulative ticks")
    ax4.set_xlabel("Time (s)")
    ax4.set_title("Encoder drift")
    ax4.grid(True, alpha=0.3)

    # ── 5. Cumulative IAE ────────────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 2])
    dt_arr = np.diff(t, prepend=t[0])
    cum_iae = np.cumsum(np.abs(m["error"]) * dt_arr)
    ax5.plot(t, cum_iae, color="#dc2626", linewidth=0.9)
    ax5.set_ylabel("Cumulative IAE (cm·s)")
    ax5.set_xlabel("Time (s)")
    ax5.set_title("Integral absolute error")
    ax5.grid(True, alpha=0.3)

    # ── 6. Metrics summary table ──────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.axis("off")
    rows = [
        ["Metric",            "Value"],
        ["RMS error",         f"{m['rms']:.3f} cm"],
        ["IAE",               f"{m['iae']:.3f} cm·s"],
        ["ITAE",              f"{m['itae']:.3f} cm·s²"],
        ["Steady-state err",  f"{m['steady_state_error']:+.2f} cm"],
        ["Overshoot",         f"{m['overshoot_pct']:.1f}%"],
        ["Undershoot",        f"{m['undershoot_pct']:.1f}%"],
        ["Rise time",         f"{m['rise_time']:.2f}s" if m['rise_time'] else "—"],
        ["Settling time",     f"{m['settling_time']:.2f}s"],
        ["Peak dist",         f"{m['peak_dist']:.1f} cm"],
        ["Trough dist",       f"{m['trough_dist']:.1f} cm"],
    ]
    tbl = ax6.table(cellText=rows[1:], colLabels=rows[0],
                    cellLoc="center", loc="center",
                    bbox=[0, 0, 1, 1])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#d1d5db")
        if r == 0:
            cell.set_facecolor("#1e3a5f")
            cell.set_text_props(color="white", fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor("#f3f4f6")
        else:
            cell.set_facecolor("white")
    ax6.set_title("Summary metrics", pad=6)

    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved → {save_path}")
    plt.show()


# ================================================================
#  METRICS PRINT
# ================================================================
def print_metrics(m):
    print("\n" + "="*50)
    print("  PERFORMANCE METRICS")
    print("="*50)
    print(f"  RMS error          : {m['rms']:.3f} cm")
    print(f"  IAE                : {m['iae']:.3f} cm·s")
    print(f"  ITAE               : {m['itae']:.3f} cm·s²")
    print(f"  Steady-state error : {m['steady_state_error']:+.2f} cm")
    print(f"  Overshoot          : {m['overshoot_pct']:.1f}%  (peak {m['peak_dist']:.1f} cm)")
    print(f"  Undershoot         : {m['undershoot_pct']:.1f}%  (trough {m['trough_dist']:.1f} cm)")
    if m["rise_time"] is not None:
        print(f"  Rise time          : {m['rise_time']:.2f} s")
    else:
        print(f"  Rise time          : never reached setpoint")
    print(f"  Settling time      : {m['settling_time']:.2f} s")
    print("="*50 + "\n")


# ================================================================
#  MAIN
# ================================================================
def main():
    parser = argparse.ArgumentParser(description="Robot follower analysis tool")
    parser.add_argument("--port", default=None,        help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--test", default="test",      help="Test name tag")
    parser.add_argument("--csv",  default=None,        help="Load CSV file instead of serial")
    parser.add_argument("--dur",  type=int, default=30, help="Recording duration in seconds")
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag   = f"{args.test}_{stamp}"

    if args.csv:
        data = read_csv(args.csv)
    elif args.port:
        csv_path = os.path.join("data", f"{tag}.csv")
        data = read_serial(args.port, args.baud, args.dur, csv_path)
    else:
        print("Provide --port for live capture or --csv for file replay.")
        print("Example:  python analyze.py --port COM3 --test step_response --dur 20")
        sys.exit(1)

    if data.size == 0:
        print("No data received.")
        sys.exit(1)

    m = compute_metrics(data)
    print_metrics(m)

    plot_path = os.path.join("plots", f"{tag}.png")
    plot_all(m, args.test.replace("_", " "), plot_path)


if __name__ == "__main__":
    main()
