import sys
import csv
import io
from pathlib import Path
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import webview
import tempfile
import os

# ── 1. Resolve log file ───────────────────────────────────────────────────────
# Logs always live two levels up from this script: ../../FC/logs/
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR    = SCRIPT_DIR.parents[0] / "FC" / "logs"

def find_log(arg: str) -> Path:
    log_files = sorted(LOG_DIR.glob("*.log"), key=lambda f: f.name)
    print("Available log files:")
    for i, log_file in enumerate(log_files):
        print(f"{i}: {log_file.name}")
    choice = int(input("Enter the number of the log file to visualize: "))
    return log_files[choice]
      
def latest_log() -> Path:
    logs = sorted(LOG_DIR.glob("*.log"), key=lambda f: f.name)
    if not logs:
        raise FileNotFoundError(f"No .log files found in {LOG_DIR}")
    return logs[-1]

# Use the latest log if no argument, otherwise try to find the specified log file.
if len(sys.argv) < 2:
    log_path = latest_log()
    print(f"No file specified — using latest log: {log_path.name}")
else:
    if sys.argv[1] == "l":
        log_path = find_log("l")
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

REQUIRED_COLS = ("ms", "thr", "sp_pitch", "sp_roll", "sp_yaw",
                 "gx", "gy", "gz", "state")

all_rows = []
for r in reader:
    try:
        # Skip rows with missing or non-numeric fields (partial serial lines)
        if any(r.get(k) is None for k in REQUIRED_COLS):
            continue
        float(r["ms"])
        # Reject corrupt rows: throttle must be a plausible PWM value (800–2200 µs)
        thr_val = float(r["thr"])
        if not (800 <= thr_val <= 2200):
            continue
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

sp_roll    = col(all_rows, "sp_roll")
sp_pitch   = col(all_rows, "sp_pitch")
sp_yaw     = col(all_rows, "sp_yaw")
throttle   = col(all_rows, "thr")
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
fig = make_subplots(
    rows=4, cols=1,
    shared_xaxes=True,
    vertical_spacing=0.08,
    subplot_titles=("setpoint roll | Gyro gx", "setpoint pitch | Gyro gy", "setpoint yaw | Gyro gz", "throttle"),
)

# ── Green RUN shading on all rows ─────────────────────────────────────────────
for x0, x1 in run_intervals:
    for row in range(1, 5):
        fig.add_vrect(x0=x0, x1=x1, fillcolor="limegreen", opacity=0.08,
                      layer="below", line_width=0, row=row, col=1)

# subplot for throttle
fig.add_trace(go.Scatter(x=t, y=throttle, name="throttle",
                         line=dict(color="orange", width=1.8)), row=4, col=1)

# ── Row 1: pid and setpoint roll ───────────────────────────────────────────────────────────────
fig.add_trace(go.Scatter(x=t, y=sp_roll, name="sp_roll",
                         line=dict(color="steelblue", width=1.8)), row=1, col=1)
fig.add_trace(go.Scatter(x=t, y=gx, name="Gyro gx (actual roll rate)",
                         line=dict(color="tomato", width=1), opacity=0.6), row=1, col=1)

# ── Row 2: Pitch ──────────────────────────────────────────────────────────────
fig.add_trace(go.Scatter(x=t, y=sp_pitch, name="sp_pitch",
                         line=dict(color="steelblue", width=1.8)), row=2, col=1)
fig.add_trace(go.Scatter(x=t, y=gy, name="Gyro gy (actual pitch rate)",
                         line=dict(color="tomato", width=1), opacity=0.6), row=2, col=1)


# ── Row 3: Gyro rates ─────────────────────────────────────────────────────────
fig.add_trace(go.Scatter(x=t, y=sp_yaw, name="sp_yaw",
                         line=dict(color="steelblue", width=1.8)), row=3, col=1)
fig.add_trace(go.Scatter(x=t, y=gz, name="Gyro gz (actual yaw rate)",
                         line=dict(color="tomato", width=1), opacity=0.6), row=3, col=1)

# confirm what unit this should be for pid 
fig.update_yaxes(title_text="deg",   row=1, col=1)
fig.update_yaxes(title_text="deg",   row=2, col=1)
fig.update_yaxes(title_text="deg/s", row=3, col=1)
fig.update_xaxes(title_text="Time (s)", row=4, col=1)
fig.update_layout(
    title=dict(text=f"PID — {log_path.name}", font=dict(size=13)),
    height=1000,
    template="plotly_dark",
)

# ── Show in native GUI window via pywebview (no browser) ──────────────────────
# Write to a temp file — WebView2's NavigateToString has a ~1.5 MB limit
# that the bundled Plotly JS easily exceeds.
with tempfile.NamedTemporaryFile(mode="w", suffix=".html", delete=False,
                                 encoding="utf-8") as f:
    f.write(fig.to_html(full_html=True, include_plotlyjs=True))
    tmp_path = f.name

try:
    url = "file:///" + tmp_path.replace("\\", "/")
    window = webview.create_window(log_path.name, url=url, width=1300, height=1020)
    webview.start()
finally:
    os.unlink(tmp_path)
