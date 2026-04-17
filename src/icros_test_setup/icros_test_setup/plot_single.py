#!/usr/bin/env python3
"""Single Experiment Plotter (논문용 스타일).

Usage: python3 plot_single.py <experiment_dir>
Output: <experiment_dir>/plots/*.png
"""
import sys
import csv
import math
from pathlib import Path
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ---- 논문용 스타일 ----
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 10,
    'axes.linewidth': 0.8,
    'grid.linewidth': 0.4,
    'lines.linewidth': 1.0,
    'xtick.direction': 'in',
    'ytick.direction': 'in',
    'figure.dpi': 150,
})

# 8방향 색상 (Tol's vibrant — colorblind-safe, 논문용)
DIR_COLORS = {
    '0.0': '#EE6677', '45.0': '#228833', '90.0': '#4477AA', '135.0': '#CCBB44',
    '180.0': '#66CCEE', '225.0': '#AA3377', '270.0': '#BBBBBB', '315.0': '#EE8866',
}
# 헤딩 오프셋별 선 스타일
HEADING_STYLES = {
    '0.0': '-',
    '-20.0': '--',
    '20.0': ':',
    '-45.0': '--',
    '45.0': ':',
}
BAR_COLOR = '#4477AA'
BAR_EDGE = '#2d5986'
# 헤딩 bar 색상 (Tol's vibrant)
HEADING_BAR_COLORS = ['#EE6677', '#4477AA', '#228833', '#AA3377']

METRIC_KEYS = [
    ('settling_time_s', 'Settling Time (s)'),
    ('steady_state_error_mm', 'Steady-State Error (mm)'),
    ('max_overshoot_mm', 'Max Overshoot (mm)'),
    ('cross_track_error_rms_mm', 'Cross-Track RMS (mm)'),
    ('heading_error_deg', 'Heading Error (deg)'),
    ('control_effort', 'Control Effort'),
    ('cdi', 'CDI (Caster Disturbance)'),
    ('steering_change_rad', 'Steering Change (rad)'),
]


def load_csv(path):
    with open(path, 'r') as f:
        return list(csv.DictReader(f))


def to_float(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def _has_multiple_headings(summary):
    headings = set(r.get('heading_offset_deg', '0.0') for r in summary)
    return len(headings) > 1


def plot_convergence(exp_dir, summary, plots_dir):
    """수렴 곡선: 방향=색, 헤딩=선스타일."""
    fig, ax = plt.subplots(figsize=(8, 4.5))
    multi_heading = _has_multiple_headings(summary)

    ts_dir = exp_dir / 'timeseries'
    labeled_dir = set()
    labeled_heading = set()
    for row in summary:
        tid = int(row['trial_id'])
        direction = row['direction_deg']
        heading = row.get('heading_offset_deg', '0.0')
        goal_x = to_float(row['goal_x'])
        goal_y = to_float(row['goal_y'])

        ts_path = ts_dir / f'trial_{tid:03d}.csv'
        if not ts_path.exists():
            continue

        ts = load_csv(ts_path)
        times = [to_float(r['t']) for r in ts]
        dists = [math.hypot(to_float(r['x']) - goal_x, to_float(r['y']) - goal_y) * 1000
                 for r in ts]

        color = DIR_COLORS.get(direction, '#666666')
        ls = HEADING_STYLES.get(heading, '-') if multi_heading else '-'

        # 범례: 방향은 색, 헤딩은 선스타일 (별도 범례)
        lbl = None
        if direction not in labeled_dir:
            lbl = f'{direction}\u00b0'
            labeled_dir.add(direction)

        ax.plot(times, dists, color=color, alpha=0.45, linewidth=0.7,
                linestyle=ls, label=lbl)

    ax.axhline(y=3, color='#cc0000', linestyle='--', linewidth=1.0, label='3 mm (success)')
    ax.axhline(y=5, color='#cc7700', linestyle=':', linewidth=0.8, label='5 mm (settle)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance to goal (mm)')
    ax.set_ylim(bottom=0)

    # 헤딩 범례 추가 (multi-heading일 때만)
    if multi_heading:
        headings = sorted(set(r.get('heading_offset_deg', '0.0') for r in summary),
                          key=to_float)
        for h in headings:
            ls = HEADING_STYLES.get(h, '-')
            ax.plot([], [], color='#333333', linestyle=ls, linewidth=1.2,
                    label=f'heading {h}\u00b0')

    ax.legend(loc='upper right', fontsize=7, framealpha=0.9, ncol=2)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(plots_dir / 'convergence.png', dpi=200)
    plt.close(fig)


def plot_direction_bars(metrics, plots_dir):
    """방향별 7지표 bar chart."""
    by_dir = defaultdict(list)
    for row in metrics:
        if row['result'] == 'SUCCESS':
            by_dir[row['direction_deg']].append(row)

    directions = sorted(by_dir.keys(), key=lambda d: to_float(d))
    dir_labels = [f"{d}\u00b0" for d in directions]

    fig, axes = plt.subplots(3, 3, figsize=(12, 9))
    axes = axes.flatten()

    for idx, (key, title) in enumerate(METRIC_KEYS):
        ax = axes[idx]
        means = []
        stds = []
        for d in directions:
            vals = [to_float(r[key]) for r in by_dir[d] if r.get(key)]
            means.append(np.mean(vals) if vals else 0)
            stds.append(np.std(vals) if vals else 0)

        x = np.arange(len(directions))
        ax.bar(x, means, yerr=stds, color=BAR_COLOR, edgecolor=BAR_EDGE,
               alpha=0.75, capsize=2, linewidth=0.5, error_kw={'linewidth': 0.7})
        ax.set_xticks(x)
        ax.set_xticklabels(dir_labels, fontsize=7)
        ax.set_title(title, fontsize=9)
        ax.grid(True, axis='y', alpha=0.2)
        ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))

    for idx in range(len(METRIC_KEYS), len(axes)):
        axes[idx].set_visible(False)

    fig.tight_layout()
    fig.savefig(plots_dir / 'direction_bars.png', dpi=200)
    plt.close(fig)


def plot_heading_bars(metrics, plots_dir):
    """헤딩 오프셋별 7지표 bar chart. 헤딩이 1개면 생략."""
    headings = sorted(set(r.get('heading_offset_deg', '0.0') for r in metrics), key=to_float)
    if len(headings) <= 1:
        return False

    by_heading = defaultdict(list)
    for row in metrics:
        if row['result'] == 'SUCCESS':
            by_heading[row.get('heading_offset_deg', '0.0')].append(row)

    h_labels = [f"{h}\u00b0" for h in headings]

    fig, axes = plt.subplots(3, 3, figsize=(12, 9))
    axes = axes.flatten()

    for idx, (key, title) in enumerate(METRIC_KEYS):
        ax = axes[idx]
        means = []
        stds = []
        for h in headings:
            vals = [to_float(r[key]) for r in by_heading[h] if r.get(key)]
            means.append(np.mean(vals) if vals else 0)
            stds.append(np.std(vals) if vals else 0)

        x = np.arange(len(headings))
        colors = HEADING_BAR_COLORS[:len(headings)]
        ax.bar(x, means, yerr=stds, color=colors, edgecolor=BAR_EDGE,
               alpha=0.75, capsize=2, linewidth=0.5, error_kw={'linewidth': 0.7})
        ax.set_xticks(x)
        ax.set_xticklabels(h_labels, fontsize=8)
        ax.set_title(title, fontsize=9)
        ax.grid(True, axis='y', alpha=0.2)
        ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))

    for idx in range(len(METRIC_KEYS), len(axes)):
        axes[idx].set_visible(False)

    fig.suptitle('Metrics by Heading Offset', fontsize=12)
    fig.tight_layout()
    fig.savefig(plots_dir / 'heading_bars.png', dpi=200)
    plt.close(fig)
    return True


def plot_heatmap(metrics, plots_dir):
    """방위(y) x 헤딩(x) settling time 히트맵. 헤딩 1개면 생략."""
    headings = sorted(set(r.get('heading_offset_deg', '0.0') for r in metrics), key=to_float)
    if len(headings) <= 1:
        return False

    directions = sorted(set(r['direction_deg'] for r in metrics), key=to_float)

    success = [r for r in metrics if r['result'] == 'SUCCESS']
    by_dh = defaultdict(list)
    for r in success:
        key = (r['direction_deg'], r.get('heading_offset_deg', '0.0'))
        by_dh[key].append(to_float(r['settling_time_s']))

    data = np.zeros((len(directions), len(headings)))
    for i, d in enumerate(directions):
        for j, h in enumerate(headings):
            vals = by_dh.get((d, h), [])
            data[i, j] = np.mean(vals) if vals else np.nan

    fig, ax = plt.subplots(figsize=(5, 6))
    im = ax.imshow(data, cmap='RdYlGn_r', aspect='auto', interpolation='nearest')
    ax.set_xticks(range(len(headings)))
    ax.set_xticklabels([f"{h}\u00b0" for h in headings])
    ax.set_yticks(range(len(directions)))
    ax.set_yticklabels([f"{d}\u00b0" for d in directions])
    ax.set_xlabel('Heading Offset')
    ax.set_ylabel('Direction')
    ax.set_title('Settling Time (s)')

    # 셀에 값 표시
    for i in range(len(directions)):
        for j in range(len(headings)):
            v = data[i, j]
            if not np.isnan(v):
                ax.text(j, i, f'{v:.2f}', ha='center', va='center', fontsize=8,
                        color='white' if v > np.nanmean(data) else 'black')

    fig.colorbar(im, ax=ax, shrink=0.8)
    fig.tight_layout()
    fig.savefig(plots_dir / 'heatmap_settling.png', dpi=200)
    plt.close(fig)
    return True


def plot_polar(metrics, plots_dir):
    """Polar plot: 방향별 settling time + SSE."""
    by_dir = defaultdict(list)
    for row in metrics:
        if row['result'] == 'SUCCESS':
            by_dir[row['direction_deg']].append(row)

    directions = sorted(by_dir.keys(), key=lambda d: to_float(d))
    angles = [math.radians(to_float(d)) for d in directions]
    angles_closed = angles + [angles[0]]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.5),
                                    subplot_kw={'projection': 'polar'})

    st_means = [np.mean([to_float(r['settling_time_s']) for r in by_dir[d]]) for d in directions]
    st_closed = st_means + [st_means[0]]
    ax1.plot(angles_closed, st_closed, 'o-', color='#333333', linewidth=1.5, markersize=4)
    ax1.fill(angles_closed, st_closed, alpha=0.15, color='#333333')
    ax1.set_title('Settling Time (s)', pad=15, fontsize=10)

    sse_means = [np.mean([to_float(r['steady_state_error_mm']) for r in by_dir[d]]) for d in directions]
    sse_closed = sse_means + [sse_means[0]]
    ax2.plot(angles_closed, sse_closed, 's-', color='#333333', linewidth=1.5, markersize=4)
    ax2.fill(angles_closed, sse_closed, alpha=0.15, color='#333333')
    ax2.set_title('Steady-State Error (mm)', pad=15, fontsize=10)

    fig.tight_layout()
    fig.savefig(plots_dir / 'polar.png', dpi=200)
    plt.close(fig)


def plot_steering_scatter(metrics, plots_dir):
    """Steering change vs Settling time scatter. 논문 핵심: 조향 많으면 수렴 느림."""
    success = [r for r in metrics if r['result'] == 'SUCCESS'
               and r.get('steering_change_rad') and r.get('settling_time_s')]
    if not success:
        return False

    sc = [to_float(r['steering_change_rad']) for r in success]
    st = [to_float(r['settling_time_s']) for r in success]

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(sc, st, c='#333333', s=30, alpha=0.7, edgecolors='#111111', linewidths=0.5)

    # 추세선
    if len(sc) >= 3:
        z = np.polyfit(sc, st, 1)
        p = np.poly1d(z)
        x_line = np.linspace(min(sc), max(sc), 50)
        ax.plot(x_line, p(x_line), '--', color='#c0392b', linewidth=1.2,
                label=f'trend (slope={z[0]:.3f})')
        # 상관계수
        corr = np.corrcoef(sc, st)[0, 1]
        ax.text(0.05, 0.95, f'r = {corr:.3f}', transform=ax.transAxes,
                fontsize=9, verticalalignment='top')
        ax.legend(fontsize=8)

    ax.set_xlabel('Steering Change (rad)')
    ax.set_ylabel('Settling Time (s)')
    ax.set_title('Steering Change vs Settling Time')
    ax.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(plots_dir / 'steering_vs_settling.png', dpi=200)
    plt.close(fig)
    return True


def _load_trial_xy(ts_dir, tid):
    """trial timeseries에서 XY/yaw를 로드. bl_x 우선, 없으면 센서 좌표 fallback."""
    ts_path = ts_dir / f'trial_{tid:03d}.csv'
    if not ts_path.exists():
        return None, None, None, False
    ts = load_csv(ts_path)
    if not ts:
        return None, None, None, False
    if 'bl_x' in ts[0] and ts[0]['bl_x'] != '':
        valid = [r for r in ts if r.get('bl_x', '') != '']
        xs = [to_float(r['bl_x']) for r in valid]
        ys = [to_float(r['bl_y']) for r in valid]
        yaws = [to_float(r['bl_yaw']) for r in valid]
        return xs, ys, yaws, True
    xs = [to_float(r['x']) for r in ts]
    ys = [to_float(r['y']) for r in ts]
    yaws = [to_float(r['yaw']) for r in ts]
    return xs, ys, yaws, False


def plot_trajectory(exp_dir, summary, plots_dir):
    """방향별 subplot으로 base_link 이동 경로 시각화.

    heading = 선 스타일, repeat = 같은 스타일로 겹침.
    모든 subplot의 축 범위를 통일하여 비교 용이."""
    ts_dir = exp_dir / 'timeseries'
    multi_heading = _has_multiple_headings(summary)

    # 1차: 전체 trial XY 로드 + 글로벌 범위 계산
    trial_data = {}  # tid -> (xs, ys,