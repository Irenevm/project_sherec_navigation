#!/usr/bin/env python3
"""
plot_telemetry.py

Offline analysis and visualization of flight telemetry produced by TelemetryLogger.
Run on the HOST (outside the Docker container) after each simulation:

    python3 mission/plot_telemetry.py                      # latest CSV in ./
    python3 mission/plot_telemetry.py path/to/file.csv     # specific file

Outputs:
    <csv_name>_analysis.png   — 4-panel figure saved next to the CSV
    Console report            — math checks (goal reached, frontier stop, giro raro)

Requirements (host):  pip install pandas matplotlib numpy
"""

import sys
import os
import glob
import math

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')   # no display needed
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D

# ─────────────────────────────────────────────────────────────────────────────
# Thresholds for automatic analysis
# ─────────────────────────────────────────────────────────────────────────────
GOAL_TOLERANCE_M     = 0.30   # drone is "at goal" if dist_to_goal < this
STOP_SPEED_THRESHOLD = 0.12   # m/s  — drone considered stopped
STOP_MIN_DURATION_S  = 0.4    # seconds of speed<threshold to count as a stop
HEADING_CHANGE_WARN  = 60.0   # degrees — flag heading jumps larger than this


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _find_latest_csv(folder: str) -> str:
    pattern = os.path.join(folder, 'sherec_nav_*.csv')
    files = sorted(glob.glob(pattern))
    if not files:
        raise FileNotFoundError(
            f'No sherec_nav_*.csv files found in {folder}')
    return files[-1]


def _load(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df['timestamp_s'] = pd.to_numeric(df['timestamp_s'])
    df['t'] = df['timestamp_s'] - df['timestamp_s'].iloc[0]   # relative time
    for col in ('pos_x', 'pos_y', 'pos_z',
                'vel_x', 'vel_y', 'vel_z', 'speed_norm', 'dist_to_goal'):
        df[col] = pd.to_numeric(df[col])
    df['event'] = df['event'].fillna('').astype(str)
    return df


def _heading_deg(df: pd.DataFrame) -> np.ndarray:
    """Heading from XY velocity in degrees, unwrapped."""
    heading = np.degrees(np.arctan2(df['vel_y'].values, df['vel_x'].values))
    return np.unwrap(heading, period=360.0)


def _detect_stops(df: pd.DataFrame) -> list[dict]:
    """
    Find segments where speed stays below threshold for >= STOP_MIN_DURATION_S.
    Returns list of dicts: {start_t, end_t, mid_t, start_idx, end_idx, mid_idx}
    """
    stops = []
    in_stop = False
    seg_start = 0

    speed = df['speed_norm'].values
    t = df['t'].values

    for i in range(len(df)):
        if not in_stop and speed[i] < STOP_SPEED_THRESHOLD:
            in_stop = True
            seg_start = i
        elif in_stop and speed[i] >= STOP_SPEED_THRESHOLD:
            duration = t[i - 1] - t[seg_start]
            if duration >= STOP_MIN_DURATION_S:
                mid = (seg_start + i - 1) // 2
                stops.append({
                    'start_t':   t[seg_start],
                    'end_t':     t[i - 1],
                    'mid_t':     t[mid],
                    'start_idx': seg_start,
                    'end_idx':   i - 1,
                    'mid_idx':   mid,
                    'duration':  duration,
                })
            in_stop = False

    if in_stop:
        duration = t[-1] - t[seg_start]
        if duration >= STOP_MIN_DURATION_S:
            mid = (seg_start + len(df) - 1) // 2
            stops.append({
                'start_t':   t[seg_start],
                'end_t':     t[-1],
                'mid_t':     t[mid],
                'start_idx': seg_start,
                'end_idx':   len(df) - 1,
                'mid_idx':   mid,
                'duration':  duration,
            })
    return stops


def _heading_jump_around_stop(df: pd.DataFrame, heading: np.ndarray,
                               stop: dict, window: int = 5) -> float:
    """
    Mean heading BEFORE the stop minus mean heading AFTER.
    Uses `window` samples on each side.
    """
    i0 = stop['start_idx']
    i1 = stop['end_idx']
    before_idx = max(0, i0 - window)
    after_idx  = min(len(df) - 1, i1 + window)

    h_before = heading[before_idx:i0].mean() if i0 > before_idx else float('nan')
    h_after  = heading[i1:after_idx].mean()  if after_idx > i1   else float('nan')

    if math.isnan(h_before) or math.isnan(h_after):
        return float('nan')
    delta = abs(h_after - h_before)
    # Normalise to [0, 180]
    return min(delta, 360.0 - delta)


# ─────────────────────────────────────────────────────────────────────────────
# Console report
# ─────────────────────────────────────────────────────────────────────────────

def _print_report(df: pd.DataFrame, stops: list[dict],
                  heading: np.ndarray, csv_path: str) -> None:
    print()
    print('=' * 62)
    print(f'  TELEMETRY ANALYSIS — {os.path.basename(csv_path)}')
    print('=' * 62)

    total_t = df['t'].iloc[-1]
    final_dist = df['dist_to_goal'].iloc[-1]
    goal = (df['goal_x'].iloc[0], df['goal_y'].iloc[0], df['goal_z'].iloc[0])

    print(f'  Goal           : ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}) m')
    print(f'  Total time     : {total_t:.1f} s')
    print(f'  Samples        : {len(df)}  ({df["t"].diff().mean()*1000:.0f} ms mean)')
    print()

    # ── Goal reached?
    goal_reached = final_dist < GOAL_TOLERANCE_M
    status = '✓  REACHED' if goal_reached else '✗  NOT REACHED'
    print(f'  Goal reached   : {status}  (final dist = {final_dist:.3f} m, tol = {GOAL_TOLERANCE_M} m)')

    # ── Stops / frontier events
    print(f'\n  Detected stops : {len(stops)}')
    for k, s in enumerate(stops):
        delta_h = _heading_jump_around_stop(df, heading, s)
        warn = ''
        if not math.isnan(delta_h) and delta_h > HEADING_CHANGE_WARN:
            warn = f'  ← ⚠ GIRO RARO ({delta_h:.1f}°)'
        print(f'    Stop #{k+1}  t=[{s["start_t"]:.1f}s … {s["end_t"]:.1f}s]  '
              f'dur={s["duration"]:.2f}s  '
              f'Δheading={delta_h:.1f}°{warn}')

    # ── Max speed
    max_speed = df['speed_norm'].max()
    print(f'\n  Max speed      : {max_speed:.2f} m/s')

    # ── Explicit events in CSV
    events = df[df['event'] != ''][['t', 'event']]
    if not events.empty:
        print(f'\n  Logged events  :')
        for _, row in events.iterrows():
            print(f'    t={row["t"]:.2f}s  {row["event"]}')

    print()
    print('=' * 62)
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Figure
# ─────────────────────────────────────────────────────────────────────────────

def _plot(df: pd.DataFrame, stops: list[dict],
          heading: np.ndarray, csv_path: str) -> str:

    goal = (df['goal_x'].iloc[0], df['goal_y'].iloc[0])
    t    = df['t'].values

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        f'Reactive Navigation Telemetry\n{os.path.basename(csv_path)}',
        fontsize=12, fontweight='bold')

    # ── colour map for speed
    speed_norm_vals = df['speed_norm'].values
    cmap   = cm.plasma
    vnorm  = Normalize(vmin=0.0, vmax=max(speed_norm_vals.max(), 0.01))

    # ─────────────────────────────────────────────
    # Subplot 1 — XY Trajectory coloured by speed
    # ─────────────────────────────────────────────
    ax1 = axes[0, 0]
    x, y = df['pos_x'].values, df['pos_y'].values

    # Draw trajectory as coloured line segments
    for i in range(len(x) - 1):
        ax1.plot(x[i:i+2], y[i:i+2],
                 color=cmap(vnorm(speed_norm_vals[i])), linewidth=1.8)

    # Start / end markers
    ax1.scatter(x[0],  y[0],  c='green', s=120, zorder=5,
                marker='^', label='Start')
    ax1.scatter(x[-1], y[-1], c='blue',  s=120, zorder=5,
                marker='s', label='End')
    ax1.scatter(*goal, c='red',  s=180, zorder=5,
                marker='*', label='Goal')

    # Stop zones
    for k, s in enumerate(stops):
        xi = df.loc[s['start_idx']:s['end_idx'], 'pos_x'].mean()
        yi = df.loc[s['start_idx']:s['end_idx'], 'pos_y'].mean()
        ax1.scatter(xi, yi, c='orange', s=80, zorder=4,
                    marker='o', edgecolors='black', linewidths=0.5)
        ax1.annotate(f'Stop#{k+1}', (xi, yi),
                     textcoords='offset points', xytext=(5, 5), fontsize=7)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=vnorm)
    sm.set_array([])
    fig.colorbar(sm, ax=ax1, label='Speed (m/s)', fraction=0.03, pad=0.04)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('XY Trajectory (colour = speed)')
    ax1.legend(fontsize=8, loc='best')
    ax1.set_aspect('equal', 'box')
    ax1.grid(True, linestyle='--', alpha=0.4)

    # ─────────────────────────────────────────────
    # Subplot 2 — Speed over time
    # ─────────────────────────────────────────────
    ax2 = axes[0, 1]
    ax2.plot(t, speed_norm_vals, color='steelblue', linewidth=1.5, label='|v|')
    ax2.axhline(STOP_SPEED_THRESHOLD, color='orange', linestyle='--',
                linewidth=1.0, label=f'Stop thr ({STOP_SPEED_THRESHOLD} m/s)')

    for s in stops:
        ax2.axvspan(s['start_t'], s['end_t'], color='orange', alpha=0.15)

    # Explicit events
    for _, row in df[df['event'] != ''].iterrows():
        ax2.axvline(row['t'], color='red', linestyle=':', linewidth=1.2, alpha=0.7)
        ax2.text(row['t'], speed_norm_vals.max() * 0.95, row['event'],
                 rotation=90, fontsize=6, color='red', va='top')

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Speed (m/s)')
    ax2.set_title('Speed over time')
    ax2.legend(fontsize=8)
    ax2.grid(True, linestyle='--', alpha=0.4)

    # ─────────────────────────────────────────────
    # Subplot 3 — Distance to goal over time
    # ─────────────────────────────────────────────
    ax3 = axes[1, 0]
    ax3.plot(t, df['dist_to_goal'].values, color='darkgreen', linewidth=1.5)
    ax3.axhline(GOAL_TOLERANCE_M, color='red', linestyle='--',
                linewidth=1.0, label=f'Goal tolerance ({GOAL_TOLERANCE_M} m)')

    for s in stops:
        ax3.axvspan(s['start_t'], s['end_t'], color='orange', alpha=0.15)

    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Distance to goal (m)')
    ax3.set_title('Distance to goal over time')
    ax3.legend(fontsize=8)
    ax3.grid(True, linestyle='--', alpha=0.4)

    # ─────────────────────────────────────────────
    # Subplot 4 — Heading + heading change rate
    # ─────────────────────────────────────────────
    ax4 = axes[1, 1]
    ax4b = ax4.twinx()

    # Mask heading where drone is nearly stopped (heading from near-zero vel is noise)
    moving_mask = speed_norm_vals > STOP_SPEED_THRESHOLD
    heading_plot = heading.copy().astype(float)
    heading_plot[~moving_mask] = np.nan

    ax4.plot(t, heading_plot, color='purple', linewidth=1.2,
             label='Heading (°)', alpha=0.85)

    # Heading change rate (°/s)
    dt = np.diff(t)
    dt[dt == 0] = np.nan
    dh = np.diff(heading)
    dh_dt = dh / dt
    # clip extreme noise
    dh_dt = np.clip(dh_dt, -360, 360)
    ax4b.plot(t[1:], dh_dt, color='tomato', linewidth=0.8,
              alpha=0.6, label='dHeading/dt (°/s)')
    ax4b.axhline(0, color='gray', linewidth=0.5)

    # Mark stop zones
    for s in stops:
        ax4.axvspan(s['start_t'], s['end_t'], color='orange', alpha=0.15)
        delta_h = _heading_jump_around_stop(df, heading, s)
        if not math.isnan(delta_h):
            ax4.annotate(
                f'Δ{delta_h:.0f}°',
                xy=(s['mid_t'], heading_plot[s['mid_idx']]
                    if not np.isnan(heading_plot[s['mid_idx']]) else 0),
                fontsize=7, color='darkorange',
                arrowprops=dict(arrowstyle='->', color='darkorange', lw=0.8),
                xytext=(s['mid_t'] + 0.5,
                        (heading_plot[~np.isnan(heading_plot)].mean()
                         if np.any(~np.isnan(heading_plot)) else 0))
            )

    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Heading (°)', color='purple')
    ax4b.set_ylabel('dHeading/dt (°/s)', color='tomato')
    ax4.set_title('Heading & heading change rate\n(NaN = drone stopped)')

    lines_a, labels_a = ax4.get_legend_handles_labels()
    lines_b, labels_b = ax4b.get_legend_handles_labels()
    ax4.legend(lines_a + lines_b, labels_a + labels_b, fontsize=8, loc='best')
    ax4.grid(True, linestyle='--', alpha=0.4)

    # ─────────────────────────────────────────────
    # Save
    # ─────────────────────────────────────────────
    plt.tight_layout()
    out_path = csv_path.replace('.csv', '_analysis.png')
    plt.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    if len(sys.argv) >= 2:
        csv_path = sys.argv[1]
    else:
        # Search in the project folder (host-side location of the Docker volume)
        search_dirs = [
            os.path.join(os.path.dirname(__file__), '..'),   # project root
            os.path.expanduser('~/project_sherec_navigation'),
        ]
        csv_path = None
        for d in search_dirs:
            try:
                csv_path = _find_latest_csv(d)
                break
            except FileNotFoundError:
                continue
        if csv_path is None:
            print('ERROR: No sherec_nav_*.csv found. Pass the path as argument.')
            sys.exit(1)

    print(f'Loading: {csv_path}')
    df = _load(csv_path)

    if df.empty:
        print('ERROR: CSV is empty.')
        sys.exit(1)

    heading = _heading_deg(df)
    stops   = _detect_stops(df)

    _print_report(df, stops, heading, csv_path)

    out_png = _plot(df, stops, heading, csv_path)
    print(f'Figure saved → {out_png}')


if __name__ == '__main__':
    main()
