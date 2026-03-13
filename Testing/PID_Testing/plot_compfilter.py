"""
Complementary filter visualizer  —  interactive matplotlib GUI
Usage:
    python plot_compfilter.py [log_name_or_path]

The log must have been captured with PRINT_CSV mode.
Omit the argument to auto-load the newest log in FC/logs/.

    Row 1  — Roll angle:  Accel (raw) vs CompFilter
    Row 2  — Pitch angle: Accel (raw) vs CompFilter
    Row 3  — Gyro rates:  gx / gy / gz  (what the rate-PID is fighting)

All rows share the same time axis (linked zoom/pan).
Green shading marks RUN-state intervals.
Use the matplotlib toolbar to zoom, pan, and save — each subplot zooms
independently when you drag over it.
"""

import sys
import csv
import io
from pathlib import Path
import matplotlib
matplotlib.use("TkAgg")          # native GUI window (zoom/pan toolbar)
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── 1. Resolve log file ───────────────────────────────────────────────────────
# Logs always live two levels up from this script: ../../FC/logs/
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR    = SCRIPT_DIR.parents[1] / "FC" / "logs"

def find_log(arg: str) -> Path:
    p = Path(arg)
    # Full or relative path that exists as-is
    if p.exists():
        return p.resolve()
    # Bare name / stem — look in the known log directory
    for suffix in ("", ".log"):
        candidate = LOG_DIR / (arg + suffix)
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        f"Cannot find '{arg}'. Looked in:\n  {p}\n  {LOG_DIR / arg}"
    )

def latest_log() -> Path:
    logs = sorted(LOG_DIR.glob("*.log"), key=lambda f: f.name)
    if not logs:
        raise FileNotFoundError(f"No .log files found in {LOG_DIR}")
    return logs[-1]

if len(sys.argv) < 2:
    log_path = latest_log()
    print(f"No file specified — using latest log: {log_path.name}")
else:
    log_path = find_log(sys.argv[1])
    print(f"Loading: {log_path}")

raw_text = log_path.read_text(encoding="utf-8", errors="replace")

def unwrap_lines(text: str) -> str:
    """Join lines that were word-wrapped by the serial monitor.
    A wrapped line ends with a comma — the next line is a continuation."""
    lines = text.splitlines()
    out = []
    buf = ""
    for ln in lines:
        stripped = ln.rstrip()
        if buf:
            buf += stripped
        else:
            buf = stripped
        if buf.endswith(","):
            continue        # continuation expected on next line
        out.append(buf)
        buf = ""
    if buf:
        out.append(buf)
    return "\n".join(out)

KNOWN_HEADER = ("ms,thr,yaw_stick,roll_stick,pitch_stick,"
                "acc_roll,acc_pitch,filt_roll,filt_pitch,"
                "gx,gy,gz,sp_roll,sp_pitch,sp_yaw,"
                "pid_roll,pid_pitch,pid_yaw,FR,FL,BR,BL,state")

header_marker = "ms,"
idx = raw_text.find(header_marker)
if idx == -1:
    # Find the first numeric-looking line and prepend the known header.
    lines = raw_text.splitlines()
    data_start = next(
        (i for i, ln in enumerate(lines) if ln.strip() and ln.strip()[0].isdigit()),
        None,
    )
    if data_start is None:
        print("ERROR: No CSV data found. Make sure the FC was running in PRINT_CSV mode.")
        sys.exit(1)
    print("Note: CSV header not in log — using known column order.")
    csv_text = KNOWN_HEADER + "\n" + "\n".join(lines[data_start:])
else:
    csv_text = raw_text[idx:]

csv_text = unwrap_lines(csv_text)
reader = csv.DictReader(io.StringIO(csv_text))

REQUIRED_COLS = ("ms", "acc_roll", "acc_pitch", "filt_roll", "filt_pitch",
                 "gx", "gy", "gz", "state")

all_rows = []
for r in reader:
    try:
        # Skip rows with missing or non-numeric fields (partial serial lines)
        if any(r.get(k) is None for k in REQUIRED_COLS):
            continue
        float(r["ms"])
        all_rows.append(r)
    except (ValueError, KeyError):
        continue

if not all_rows:
    print("ERROR: No valid CSV rows found.")
    sys.exit(1)

# ── 2. Parse columns (all rows, colour-code state) ───────────────────────────
def col(rows, key):
    return [float(r[key]) for r in rows]

t0 = float(all_rows[0]["ms"]) / 1000.0
t  = [float(r["ms"]) / 1000.0 - t0 for r in all_rows]

acc_roll   = col(all_rows, "acc_roll")
acc_pitch  = col(all_rows, "acc_pitch")
filt_roll  = col(all_rows, "filt_roll")
filt_pitch = col(all_rows, "filt_pitch")
gx         = col(all_rows, "gx")
gy         = col(all_rows, "gy")
gz         = col(all_rows, "gz")
state      = [r.get("state", "").strip() for r in all_rows]

# ── 3. Find RUN intervals for green shading ───────────────────────────────────
run_intervals = []
in_run = False
for i, s in enumerate(state):
    if s == "RUN" and not in_run:
        run_start = t[i]
        in_run = True
    elif s != "RUN" and in_run:
        run_intervals.append((run_start, t[i - 1]))
        in_run = False
if in_run:
    run_intervals.append((run_start, t[-1]))

# ── 4. Build figure ───────────────────────────────────────────────────────────
plt.style.use("seaborn-v0_8-whitegrid")
fig, (ax1, ax2, ax3) = plt.subplots(
    3, 1, figsize=(13, 9),
    sharex=True,
    gridspec_kw=dict(hspace=0.35),
)
fig.suptitle(f"Complementary Filter Verification\n{log_path.name}", fontsize=13, fontweight="bold")
fig.canvas.manager.set_window_title(str(log_path.name))

# ── Green RUN shading on all axes ─────────────────────────────────────────────
for x0, x1 in run_intervals:
    for ax in (ax1, ax2, ax3):
        ax.axvspan(x0, x1, color="limegreen", alpha=0.08, linewidth=0,
                   label="_nolegend_")

# ── Row 1: Roll ───────────────────────────────────────────────────────────────
ax1.plot(t, acc_roll,  color="tomato",    alpha=0.6, linewidth=1,   label="acc_roll  (raw)")
ax1.plot(t, filt_roll, color="steelblue", linewidth=1.8,             label="filt_roll (comp filter)")
ax1.set_ylabel("deg")
ax1.set_title("Roll angle")
ax1.legend(fontsize=8, loc="upper right")

# ── Row 2: Pitch ──────────────────────────────────────────────────────────────
ax2.plot(t, acc_pitch,  color="tomato",    alpha=0.6, linewidth=1,   label="acc_pitch  (raw)")
ax2.plot(t, filt_pitch, color="steelblue", linewidth=1.8,             label="filt_pitch (comp filter)")
ax2.set_ylabel("deg")
ax2.set_title("Pitch angle")
ax2.legend(fontsize=8, loc="upper right")

# ── Row 3: Gyro rates ─────────────────────────────────────────────────────────
ax3.plot(t, gx, color="darkorange",    linewidth=1.2, label="gx")
ax3.plot(t, gy, color="mediumseagreen",linewidth=1.2, label="gy")
ax3.plot(t, gz, color="mediumpurple",  linewidth=1.2, label="gz")
ax3.axhline(0, color="black", linewidth=0.6, linestyle="--")
ax3.set_ylabel("deg/s")
ax3.set_xlabel("Time (s)")
ax3.set_title("Gyro rates")
ax3.legend(fontsize=8, loc="upper right")

# ── Shared x-axis tick formatting ─────────────────────────────────────────────
ax3.xaxis.set_major_locator(ticker.MultipleLocator(5))
ax3.xaxis.set_minor_locator(ticker.MultipleLocator(1))

plt.show()
