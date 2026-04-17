#!/usr/bin/env python3
"""Multi-Experiment Comparison Plotter (논문용 스타일).

Usage:
    python3 plot_compare.py <exp_dir_1> <exp_dir_2> [exp_dir_3 ...]

Output:
    comparison_<timestamp>/
    ├── boxplots.png                    — 지표별 box plot (분포 비교)
    ├── radar.png                       — 정규화 radar chart
    ├── convergence_compare.png         — 대표 방향 수렴 곡선 비교
    ├── success_rate.png                — 성공률 bar
    ├── heading_robustness.png          — 헤딩 강건성 (헤딩 2종 이상 시)
    ├── steering_vs_settling.png        — 조향-수렴 scatter
    └── comparison.csv                  — 비교 요약 테이블
"""
import sys
import csv
import math
from pathlib import Path
from datetime import datetime
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

CTRL_COLORS = ['#EE6677', '#4477AA', '#228833', '#AA3377', '#EE8866']
CTRL_MARKERS = ['o', 's', '^', 'D', 'v']
CTRL_HATCHES = ['///', '...', 'xxx', '\\\\\\', '+++']

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

REPRESENTATIVE_DIRS = ['0.0', '90.0', '180.0', '270.0']


def load_csv(path):
    with open(path, 'r') as f:
        return list(csv.DictReader(f))


def to_float(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def _load_success_metrics(experiments):
    """컨트롤러별 성공 trial metrics를 dict로 반환."""
    ctrl_metrics = {}
    for name, exp_dir in experiments:
        metrics_path = exp_dir / 'metrics.csv'
        if metrics_path.exists():
            rows = load_csv(metrics_path)
            ctrl_metrics[name] = [r for r in rows if r['result'] == 'SUCCESS']
        else:
            ctrl_metrics[name] = []
    return ctrl_metrics


# ====================================================================
# 1. Box plots — 7지표 분포 비교
# ====================================================================
def plot_boxplots(experiments, out_dir):
    """지표별 box plot. 흑백 인쇄 대비 해치 패턴."""
    ctrl_metrics = _load_success_metrics(experiments)
    names = [n for n, _ in experiments]

    fig, axes = plt.subplots(3, 3, figsize=(13, 9))
    axes = axes.flatten()

    for m_idx, (key, title) in enumerate(METRIC_KEYS):
        ax = axes[m_idx]
        data = []
        for name in names:
            vals = [to_float(r[key]) for r in ctrl_metrics[name] if r.get(key)]
            data.append(vals if vals else [0])

        bp = ax.boxplot(data, labels=names, patch_artist=True,
                        widths=0.5, showfliers=True,
                        flierprops={'markersize': 3, 'alpha': 0.5})

        for i, patch in enumerate(bp['boxes']):
            color = CTRL_COLORS[i % len(CTRL_COLORS)]
            patch.set_facecolor(color)
            patch.set_alpha(0.6)
            patch.set_hatch(CTRL_HATCHES[i % len(CTRL_HATCHES)])
            patch.set_edgecolor('#333333')

        ax.set_title(title, fontsize=9)
        ax.grid(True, axis='y', alpha=0.2)
        ax.tick_params(axis='x', labelsize=8)

    for idx in range(len(METRIC_KEYS), len(axes)):
        axes[idx].set_visible(False)

    fig.tight_layout()
    fig.savefig(out_dir / 'boxplots.png', dpi=200)
    plt.close(fig)


# ====================================================================
# 2. Radar chart — 정규화 7지표 오버레이
# ====================================================================
def plot_radar(experiments, out_dir):
    """정규화 radar chart. 면적 클수록 전반적 성능 우수."""
    ctrl_metrics = _load_success_metrics(experiments)
    names = [n for n, _ in experiments]
    labels = [title for _, title in METRIC_KEYS]
    n_metrics = len(METRIC_KEYS)

    # 컨트롤러별 평균
    ctrl_means = {}
    for name in names:
        means = []
        for key, _ in METRIC_KEYS:
            vals = [to_float(r[key]) for r in ctrl_metrics[name] if r.get(key)]
            means.append(np.mean(vals) if vals else 0)
        ctrl_means[name] = means

    # 정규화: 각 지표에서 min=1(최고), max=0(최저)
    all_means = np.array([ctrl_means[n] for n in names])
    if len(all_means) < 2:
        return False
    mins = all_means.min(axis=0)
    maxs = all_means.max(axis=0)
    ranges = maxs - mins
    ranges[ranges == 0] = 1  # division by zero 방지

    normalized = {}
    for name in names:
        normalized[name] = [1.0 - (v - mn) / rng
                            for v, mn, rng in zip(ctrl_means[name], mins, ranges)]

    # Radar plot
    angles = np.linspace(0, 2 * np.pi, n_metrics, endpoint=False).tolist()
    angles += angles[:1]

    fig, ax = plt.subplots(figsize=(7, 7), subplot_kw={'projection': 'polar'})

    for idx, name in enumerate(names):
        vals = normalized[name] + [normalized[name][0]]
        color = CTRL_COLORS[idx % len(CTRL_COLORS)]
        marker = CTRL_MARKERS[idx % len(CTRL_MARKERS)]
        ax.plot(angles, vals, f'{marker}-', color=color, linewidth=1.5,
                markersize=5, label=name)
        ax.fill(angles, vals, alpha=0.1, color=color)

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylim(0, 1.1)
    ax.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(['0.25', '0.50', '0.75', '1.00'], fontsize=7)
    ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1.1), fontsize=9)
    ax.set_title('Normalized Metrics (higher = better)', pad=20, fontsize=11)

    fig.tight_layout()
    fig.savefig(out_dir / 'radar.png', dpi=200)
    plt.close(fig)
    return True


# ====================================================================
# 3. Convergence compare — 대표 방향 수렴 곡선
# ====================================================================
def plot_convergence_compare(experiments, out_dir):
    """대표 방향(0, 90, 180, 270)에서 컨트롤러별 수렴 곡선 오버레이."""
    # 실제 존재하는 방향만 사용
    all_dirs = set()
    for _, exp_dir in experiments:
        summary = load_csv(exp_dir / 'summary.csv')
        all_dirs.update(r['direction_deg'] for r in summary)

    rep_dirs = [d for d in REPRESENTATIVE_DIRS if d in all_dirs]
    if not rep_dirs:
        rep_dirs = sorted(all_dirs, key=to_float)[:4]

    n_dirs = len(rep_dirs)
    cols = min(n_dirs, 2)
    rows = math.ceil(n_dirs / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 4 * rows), squeeze=False)

    for d_idx, direction in enumerate(rep_dirs):
        ax = axes[d_idx // cols][d_idx % cols]
        ax.set_title(f'Direction {direction}\u00b0', fontsize=10)

        for c_idx, (name, exp_dir) in enumerate(experiments):
            color = CTRL_COLORS[c_idx % len(CTRL_COLORS)]
            summary = load_csv(exp_dir / 'summary.csv')
            ts_dir = exp_dir / 'timeseries'
            labeled = False

            for row in summary:
                if row['direction_deg'] != direction:
                    continue
                tid = int(row['trial_id'])
                goal_x = to_float(row['goal_x'])
                goal_y = to_float(row['goal_y'])

                ts_path = ts_dir / f'trial_{tid:03d}.csv'
                if not ts_path.exists():
                    continue

                ts = load_csv(ts_path)
                times = [to_float(r['t']) for r in ts]
                dists = [math.hypot(to_float(r['x']) - goal_x,
                                    to_float(r['y']) - goal_y) * 1000
                         for r in ts]

                lbl = name if not labeled else None
                ax.plot(times, dists, color=color, alpha=0.5, linewidth=0.8,
                        label=lbl)
                labeled = True

        ax.axhline(y=3, color='#cc0000', linestyle='--', linewidth=0.8,
                   label='3 mm' if d_idx == 0 else None)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (mm)')
        ax.set_ylim(bottom=0)
        ax.grid(True, alpha=0.25)
        if d_idx == 0:
            ax.legend(fontsize=7, loc='upper right')

    # 남는 subplot 숨기기
    for d_idx in range(n_dirs, rows * cols):
        axes[d_idx // cols][d_idx % cols].set_visible(False)

    fig.tight_layout()
    fig.savefig(out_dir / 'convergence_compare.png', dpi=200)
    plt.close(fig)


# ====================================================================
# 4. Success rate bar
# ====================================================================
def plot_success_rate(experiments, out_dir):
    """컨트롤러별 성공률 bar chart."""
    names = []
    rates = []
    totals = []

    for name, exp_dir in experiments:
        metrics_path = exp_dir / 'metrics.csv'
        if not metrics_path.exists():
            continue
        all_rows = load_csv(metrics_path)
        total = len(all_rows)
        n_success = sum(1 for r in all_rows if r['result'] == 'SUCCESS')
        names.append(name)
        rates.append(n_success / max(total, 1) * 100)
        totals.append(total)

    fig, ax = plt.subplots(figsize=(5, 4))
    x = np.arange(len(names))
    bars = ax.bar(x, rates, color=CTRL_COLORS[:len(names)], alpha=0.8,
                  edgecolor='#333333', linewidth=0.8)

    for i, (bar, rate, total) in enumerate(zip(bars, rates, totals)):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                f'{rate:.1f}%\n({int(rate * total / 100)}/{total})',
                ha='center', va='bottom', fontsize=9)

    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=10)
    ax.set_ylabel('Success Rate (%)')
    ax.set_ylim(0, 115)
    ax.grid(True, axis='y', alpha=0.2)
    ax.set_title('Success Rate by Controller')

    fig.tight_layout()
    fig.savefig(out_dir / 'success_rate.png', dpi=200)
    plt.close(fig)


# ====================================================================
# 5. Heading robustness — 헤딩별 settling time 비교
# ====================================================================
def plot_heading_robustness(experiments, out_dir):
    """헤딩 오프셋별 settling time 비교. 헤딩 1종이면 생략."""
    ctrl_metrics = _load_success_metrics(experiments)
    names = [n for n, _ in experiments]

    all_headings = set()
    for name in names:
        all_headings.update(r.get('heading_offset_deg', '0.0')
                            for r in ctrl_metrics[name])
    if len(all_headings) <= 1:
        return False

    headings = sorted(all_headings, key=to_float)

    fig, ax = plt.subplots(figsize=(8, 5))
    n_ctrl = len(names)
    bar_width = 0.7 / n_ctrl
    x = np.arange(len(headings))

    for c_idx, name in enumerate(names):
        by_h = defaultdict(list)
        for r in ctrl_metrics[name]:
            h = r.get('heading_offset_deg', '0.0')
            if r.get('settling_time_s'):
                by_h[h].append(to_float(r['settling_time_s']))

        means = [np.mean(by_h[h]) if by_h[h] else 0 for h in headings]
        stds = [np.std(by_h[h]) if by_h[h] else 0 for h in headings]

        offset = (c_idx - (n_ctrl - 1) / 2) * bar_width
        color = CTRL_COLORS[c_idx % len(CTRL_COLORS)]
        ax.bar(x + offset, means, bar_width, yerr=stds,
               color=color, alpha=0.75, capsize=3,
               label=name, edgecolor='#333333', linewidth=0.5,
               error_kw={'linewidth': 0.7})

    ax.set_xticks(x)
    ax.set_xticklabels([f'{h}\u00b0' for h in headings])
    ax.set_xlabel('Heading Offset')
    ax.set_ylabel('Settling Time (s)')
    ax.set_title('Heading Robustness: Settling Time by Heading Offset')
    ax.legend(fontsize=8)
    ax.grid(True, axis='y', alpha=0.2)

    fig.tight_layout()
    fig.savefig(out_dir / 'heading_robustness.png', dpi=200)
    plt.close(fig)
    return True


# ====================================================================
# 6. Steering vs Settling scatter — 조향-수렴 상관
# ====================================================================
def plot_steering_scatter(experiments, out_dir):
    """Steering change vs Settling time scatter. 컨트롤러별 군집 + 전체 추세선."""
    fig, ax = plt.subplots(figsize=(7, 5.5))
    all_sc, all_st = [], []
    any_data = False

    for idx, (name, exp_dir) in enumerate(experiments):
        metrics_path = exp_dir / 'metrics.csv'
        if not metrics_path.exists():
            continue
        success = [r for r in load_csv(metrics_path)
                   if r['result'] == 'SUCCESS'
                   and r.get('steering_change_rad')
                   and r.get('settling_time_s')]
        if not success:
            continue

        sc = [to_float(r['steering_change_rad']) for r in success]
        st = [to_float(r['settling_time_s']) for r in success]
        color = CTRL_COLORS[idx % len(CTRL_COLORS)]
        marker = CTRL_MARKERS[idx % len(CTRL_MARKERS)]

        ax.scatter(sc, st, c=color, marker=marker, s=35, alpha=0.7,
                   edgecolors='#111111', linewidths=0.3, label=name, zorder=3)
        all_sc.extend(sc)
        all_st.extend(st)
        any_data = True

    if not any_data:
        plt.close(fig)
        return False

    # 전체 추세선
    if len(all_sc) >= 3:
        z = np.polyfit(all_sc, all_st, 1)
        p = np.poly1d(z)
        x_line = np.linspace(min(all_sc), max(all_sc), 50)
        ax.plot(x_line, p(x_line), '--', color='#555555', linewidth=1.2,
                label=f'trend (slope={z[0]:.3f})', zorder=2)
        corr = np.corrcoef(all_sc, all_st)[0, 1]
        ax.text(0.05, 0.95, f'r = {corr:.3f}', transform=ax.transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                          edgecolor='#cccccc', alpha=0.9))

    ax.set_xlabel('Steering Change (rad)')
    ax.set_ylabel('Settling Time (s)')
    ax.set_title('Steering Change vs Settling Time')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_dir / 'steering_vs_settling.png', dpi=200)
    plt.close(fig)
    return True


# ====================================================================
# 7. CDI vs SSE scatter — 캐스터 교란-정밀도 상관
# ====================================================================
def plot_cdi_scatter(experiments, out_dir):
    """CDI vs Steady-State Error scatter. 전체 trial (성공+실패) 포함."""
    fig, ax = plt.subplots(figsize=(7, 5.5))
    all_cdi, all_sse = [], []
    any_data = False

    for idx, (name, exp_dir) in enumerate(experiments):
        metrics_path = exp_dir / 'metrics.csv'
        if not metrics_path.exists():
            continue
        rows = [r for r in load_csv(metrics_path)
                if r.get('cdi') and r.get('steady_state_error_mm')]
        if not rows:
            continue

        cdi = [to_float(r['cdi']) for r in rows]
        sse = [to_float(r['steady_state_error_mm']) for r in rows]
        color = CTRL_COLORS[idx % len(CTRL_COLORS)]
        marker = CTRL_MARKERS[idx % len(CTRL_MARKERS)]

        ax.scatter(cdi, sse, c=color, marker=marker, s=35, alpha=0.7,
                   edgecolors='#111111', linewidths=0.3, label=name, zorder=3)
        all_cdi.extend(cdi)
        all_sse.extend(sse)
        any_data = True

    if not any_data:
        plt.close(fig)
        return False

    # 전체 추세선
    if len(all_cdi) >= 3:
        z = np.polyfit(all_cdi, all_sse, 1)
        p = np.poly1d(z)
        x_line = np.linspace(min(all_cdi), max(all_cdi), 50)
        ax.plot(x_line, p(x_line), '--', color='#555555', linewidth=1.2,
                label=f'trend (slope={z[0]:.3f})', zorder=2)
        corr = np.corrcoef(all_cdi, all_sse)[0, 1]
        ax.text(0.05, 0.95, f'r = {corr:.3f}', transform=ax.transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                          edgecolor='#cccccc', alpha=0.9))

    ax.set_xlabel('CDI (Caster Disturbance Index)')
    ax.set_ylabel('Steady-State Error (mm)')
    ax.set_title('CDI vs Steady-State Error')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_dir / 'cdi_vs_sse.png', dpi=200)
    plt.close(fig)
    return True


# ====================================================================
# 8. Comparison CSV — 요약 테이블
# ====================================================================
def write_comparison_csv(experiments, out_dir):
    """컨트롤러별 7지표 mean/std/min/max CSV."""
    ctrl_metrics = _load_success_metrics(experiments)
    names = [n for n, _ in experiments]

    fields = ['controller', 'n_total', 'n_success', 'success_rate']
    for key, _ in METRIC_KEYS:
        fields.extend([f'{key}_mean', f'{key}_std', f'{key}_min', f'{key}_max'])

    rows = []
    for name, exp_dir in experiments:
        metrics_path = exp_dir / 'metrics.csv'
        all_rows = load_csv(metrics_path) if metrics_path.exists() else []
        total = len(all_rows)
        n_success = sum(1 for r in all_rows if r['result'] == 'SUCCESS')
        success = ctrl_metrics[name]

        row = {
            'controller': name,
            'n_total': total,
            'n_success': n_success,
            'success_rate': f'{n_success / max(total, 1) * 100:.1f}',
        }

        for key, _ in METRIC_KEYS:
            vals = [to_float(r[key]) for r in success if r.get(key)]
            if vals:
                row[f'{key}_mean'] = f'{np.mean(vals):.4f}'
                row[f'{key}_std'] = f'{np.std(vals):.4f}'
                row[f'{key}_min'] = f'{np.min(vals):.4f}'
                row[f'{key}_max'] = f'{np.max(vals):.4f}'
            else:
                for suffix in ('_mean', '_std', '_min', '_max'):
                    row[f'{key}{suffix}'] = ''

        rows.append(row)

    csv_path = out_dir / 'comparison.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


# ====================================================================
# Main
# ====================================================================
def main():
    if len(sys.argv) < 3:
        print("Usage: python3 plot_compare.py <exp_dir_1> <exp_dir_2> [exp_dir_3 ...]")
        sys.exit(1)

    experiments = []
    for arg in sys.argv[1:]:
        exp_dir = Path(arg)
        if not exp_dir.exists():
            print(f"Error: {exp_dir} not found")
            sys.exit(1)
        if not (exp_dir / 'metrics.csv').exists():
            print(f"Error: metrics.csv not found in {exp_dir}")
            sys.exit(1)
        name = exp_dir.name.split('_')[0]
        experiments.append((name, exp_dir))

    # 공통 부모 디렉토리 아래에 comparison 생성
    parent = Path(experiments[0][1]).parent
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = parent / f'comparison_{timestamp}'
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Comparing: {', '.join(n for n, _ in experiments)}")
    print(f"Output: {out_dir}/\n")

    plot_boxplots(experiments, out_dir)
    print("  boxplots.png")

    if plot_radar(experiments, out_dir):
        print("  radar.png")

    plot_convergence_compare(experiments, out_dir)
    print("  convergence_compare.png")

    plot_success_rate(experiments, out_dir)
    print("  success_rate.png")

    if plot_heading_robustness(experiments, out_dir):
        print("  heading_robustness.png")

    if plot_steering_scatter(experiments, out_dir):
        print("  steering_vs_settling.png")

    if plot_cdi_scatter(experiments, out_dir):
        print("  cdi_vs_sse.png")

    csv_path = write_comparison_csv(experiments, out_dir)
    print(f"  comparison.csv")

    print(f"\nDone. Comparison saved to {out_dir}/")


if __name__ == '__main__':
    main()
