"""Experiment Analyzer.

실험 결과 디렉토리를 입력받아 7개 평가 지표를 계산하고 리포트를 생성한다.

Usage (standalone):
    python3 -m icros_test_setup.experiment_analyzer <experiment_dir>

Usage (from code):
    from icros_test_setup.experiment_analyzer import analyze
    analyze('/path/to/kinematic_20260414_103943')
"""
import sys
import csv
import math
from pathlib import Path


def load_csv(path):
    with open(path, 'r') as f:
        return list(csv.DictReader(f))


def to_float(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def compute_metrics(summary_row, ts_rows):
    """trial 하나에 대한 7개 지표를 계산한다."""
    goal_x = to_float(summary_row['goal_x'])
    goal_y = to_float(summary_row['goal_y'])
    goal_yaw = math.radians(to_float(summary_row.get('goal_yaw_deg', 0)))
    start_x = to_float(summary_row['start_x'])
    start_y = to_float(summary_row['start_y'])

    if not ts_rows:
        return None

    # Parse timeseries
    ts = []
    for row in ts_rows:
        ts.append({
            't': to_float(row['t']),
            'x': to_float(row['x']),
            'y': to_float(row['y']),
            'yaw': to_float(row['yaw']),
            'vx': to_float(row['vx']),
            'vy': to_float(row['vy']),
            'wz': to_float(row['wz']),
        })

    # Joint columns — all non-base fields
    base_keys = {'t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz'}
    all_joint_cols = sorted(k for k in ts_rows[0] if k not in base_keys)
    # Steering joints only (이름에 'steering' 포함, 또는 legacy j0/j2)
    steering_cols = [c for c in all_joint_cols if 'steering' in c]
    if not steering_cols:
        # legacy: j0=front_steering, j2=rear_steering
        steering_cols = [c for c in all_joint_cols if c in ('j0', 'j2')]
    for i, row in enumerate(ts_rows):
        for jc in all_joint_cols:
            ts[i][jc] = to_float(row[jc])

    def dist_to_goal(p):
        return math.hypot(p['x'] - goal_x, p['y'] - goal_y)

    # --- 1. Settling time ---
    settling_time = None
    for p in ts:
        if dist_to_goal(p) < 0.003:
            settling_time = p['t']
            break

    # --- 2. Steady-state error (최초 3mm 도달 이후 평균 오차) ---
    settled_samples = []
    reached = False
    for p in ts:
        if not reached and dist_to_goal(p) < 0.003:
            reached = True
        if reached:
            settled_samples.append(dist_to_goal(p))
    if settled_samples:
        sse = sum(settled_samples) / len(settled_samples)
    else:
        # 3mm 도달 못한 경우: 마지막 1초 평균
        t_end = ts[-1]['t']
        last_1s = [p for p in ts if p['t'] >= t_end - 1.0]
        sse = sum(dist_to_goal(p) for p in last_1s) / len(last_1s) if last_1s else dist_to_goal(ts[-1])

    # --- 3. Max overshoot ---
    max_overshoot = 0.0
    passed = False
    for p in ts:
        d = dist_to_goal(p)
        if not passed and d < 0.003:
            passed = True
        if passed:
            if d > max_overshoot:
                max_overshoot = d

    # --- 4. Cross-track error (RMS) ---
    dx = goal_x - start_x
    dy = goal_y - start_y
    line_len = math.hypot(dx, dy)
    if line_len > 1e-6:
        ct_errors = []
        for p in ts:
            px = p['x'] - start_x
            py = p['y'] - start_y
            cross = abs(px * dy - py * dx) / line_len
            ct_errors.append(cross ** 2)
        cte_rms = math.sqrt(sum(ct_errors) / len(ct_errors))
    else:
        cte_rms = 0.0

    # --- 5. Heading error (final) ---
    heading_err = abs(ts[-1]['yaw'] - goal_yaw)
    heading_err = abs((heading_err + math.pi) % (2 * math.pi) - math.pi)

    # --- 6. Control effort ---
    control_effort = 0.0
    for i in range(1, len(ts)):
        dvx = ts[i]['vx'] - ts[i - 1]['vx']
        dvy = ts[i]['vy'] - ts[i - 1]['vy']
        control_effort += math.hypot(dvx, dvy)

    # --- 7. Caster Disturbance Index (CDI) ---
    # CDI = (1/N) * Σ( |Δθ_steer[i]| / (v_wheel[i] + ε) )
    # 저속에서의 조향 변화가 캐스터에 미치는 교란을 정량화.
    EPSILON = 0.001  # m/s — 이 이하에서는 캐스터 자연 정렬 불가
    # wheel joints (이름에 'wheel' 포함)
    wheel_cols = [c for c in all_joint_cols if 'wheel' in c]
    cdi = 0.0
    steering_change = 0.0
    n_samples = len(ts) - 1
    if steering_cols and wheel_cols and n_samples > 0:
        for i in range(1, len(ts)):
            # 조향 변화량 (전후방 평균)
            delta_steer = sum(abs(ts[i][jc] - ts[i - 1][jc])
                              for jc in steering_cols) / len(steering_cols)
            steering_change += delta_steer

            # 휠 선속도 (wheel joint 각속도 × 휠 반경 추정)
            # joint value = 누적 각도 → 차분 / dt = 각속도
            dt = ts[i]['t'] - ts[i - 1]['t']
            if dt > 0:
                wheel_speed = sum(abs(ts[i][wc] - ts[i - 1][wc]) / dt
                                  for wc in wheel_cols) / len(wheel_cols)
                # 대략적 휠 반경 0.05m 적용 (선속도 = 각속도 × r)
                v_wheel = wheel_speed * 0.05
            else:
                v_wheel = 0.0

            cdi += delta_steer / (v_wheel + EPSILON)

        cdi /= n_samples  # 시간 정규화

    return {
        'settling_time_s': round(settling_time, 4) if settling_time is not None else None,
        'steady_state_error_mm': round(sse * 1000, 3),
        'max_overshoot_mm': round(max_overshoot * 1000, 3),
        'cross_track_error_rms_mm': round(cte_rms * 1000, 3),
        'heading_error_deg': round(math.degrees(heading_err), 2),
        'control_effort': round(control_effort, 6),
        'cdi': round(cdi, 4),
        'steering_change_rad': round(steering_change, 4),
    }


def analyze(experiment_dir):
    """메인 분석 함수. experiment_dir 경로를 받아 metrics.csv + report.txt 생성."""
    exp_dir = Path(experiment_dir)
    summary_path = exp_dir / 'summary.csv'
    ts_dir = exp_dir / 'timeseries'

    if not summary_path.exists():
        print(f"Error: {summary_path} not found")
        return

    summary = load_csv(summary_path)
    print(f"Loaded {len(summary)} trials from {summary_path}")

    metrics_fields = [
        'trial_id', 'controller', 'result',
        'direction_deg', 'heading_offset_deg', 'repeat',
        'settling_time_s', 'steady_state_error_mm', 'max_overshoot_mm',
        'cross_track_error_rms_mm', 'heading_error_deg',
        'control_effort', 'cdi', 'steering_change_rad',
    ]

    all_metrics = []
    for row in summary:
        trial_id = row['trial_id']
        ts_path = ts_dir / f'trial_{int(trial_id):03d}.csv'

        if ts_path.exists():
            ts_rows = load_csv(ts_path)
            m = compute_metrics(row, ts_rows)
        else:
            print(f"  Warning: {ts_path} not found, skipping metrics")
            m = None

        entry = {
            'trial_id': trial_id,
            'controller': row['controller'],
            'direction_deg': row['direction_deg'],
            'heading_offset_deg': row.get('heading_offset_deg', '0.0'),
            'repeat': row['repeat'],
            'result': row['result'],
        }
        if m:
            entry.update(m)
        all_metrics.append(entry)

    # Write metrics CSV
    metrics_path = exp_dir / 'metrics.csv'
    with open(metrics_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=metrics_fields)
        writer.writeheader()
        for entry in all_metrics:
            writer.writerow({k: entry.get(k, '') for k in metrics_fields})
    print(f"Metrics written to {metrics_path}")

    # Generate report
    report_lines = []
    report_lines.append(f"=== Experiment Report: {exp_dir.name} ===\n")

    success_trials = [m for m in all_metrics
                      if m['result'] == 'SUCCESS' and m.get('settling_time_s') is not None]
    total = len(all_metrics)
    n_success = len([m for m in all_metrics if m['result'] == 'SUCCESS'])
    report_lines.append(f"Trials: {total}, Success: {n_success}, "
                        f"Rate: {n_success / max(total, 1) * 100:.1f}%\n")

    if success_trials:
        def stat(key, unit=''):
            vals = [m[key] for m in success_trials if m.get(key) is not None]
            if not vals:
                return f"  {key}: no data"
            avg = sum(vals) / len(vals)
            mn = min(vals)
            mx = max(vals)
            return f"  {key}: avg={avg:.3f}{unit}, min={mn:.3f}{unit}, max={mx:.3f}{unit}"

        report_lines.append("--- Metrics (success trials only) ---")
        report_lines.append(stat('settling_time_s', 's'))
        report_lines.append(stat('steady_state_error_mm', 'mm'))
        report_lines.append(stat('max_overshoot_mm', 'mm'))
        report_lines.append(stat('cross_track_error_rms_mm', 'mm'))
        report_lines.append(stat('heading_error_deg', 'deg'))
        report_lines.append(stat('control_effort'))
        report_lines.append(stat('cdi'))
        report_lines.append(stat('steering_change_rad', 'rad'))

        report_lines.append("\n--- Per Direction ---")
        directions = sorted(set(m['direction_deg'] for m in success_trials))
        for d in directions:
            d_trials = [m for m in success_trials if m['direction_deg'] == d]
            avg_st = sum(m['settling_time_s'] for m in d_trials) / len(d_trials)
            avg_sse = sum(m['steady_state_error_mm'] for m in d_trials) / len(d_trials)
            report_lines.append(
                f"  dir={d:>5s} deg: n={len(d_trials)}, "
                f"settling={avg_st:.3f}s, sse={avg_sse:.3f}mm")

        # Per heading (only if multiple headings exist)
        headings = sorted(set(m['heading_offset_deg'] for m in success_trials))
     