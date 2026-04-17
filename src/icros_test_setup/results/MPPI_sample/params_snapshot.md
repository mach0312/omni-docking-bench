# Parameter Snapshot — MPPI Phase 2

## 실험: MPPI_20260416_074529_simulation

## 단계: Phase 2 Heading Sweep (heading 0°,±2°,±3°,±4°,±5°, 2 repeats, 144 trials)

### experiment_runner 파라미터

- controller_name: MPPI
- heading_offsets_deg: [0.0, -2.0, 2.0, -3.0, 3.0, -4.0, 4.0, -5.0, 5.0]
- repeats: 2
- success_threshold: 0.003m
- success_samples: 5
- settle_timeout: 60s
- total_timeout: 90s
- post_controller_timeout: 5s
- sensor_offset: (0.0, 0.51)
- 출발점: base_link 기준 (bl_goal 역산 적용)
- 모니터링: \_monitor_trial_action (controller 완료 후 센서 수렴 판정)

### test_experiments.yaml — MPPI 섹션 (Phase 1과 동일)

```yaml
MPPI:
  plugin: nav2_mppi_controller::MPPIController
  time_steps: 20
  model_dt: 0.05
  batch_size: 2000
  vx_std: 0.01
  vy_std: 0.01
  wz_std: 0.02
  vx_max: 0.1
  vy_max: 0.1
  wz_max: 0.2
  iteration_count: 2
  temperature: 0.3
  gamma: 0.015
  motion_model: Omni

  critics:
    - ConstraintCritic (w=4.0)
    - CostCritic (w=3.81)
    - GoalCritic (w=20.0, threshold=0.5)
    - GoalAngleCritic (w=10.0, threshold=0.2)
    - VelocityDeadbandCritic (w=10.0, deadband=[0.001,0.001,0.001])
```

### test_experiments.yaml — goal_checker

```yaml
goal_checker:
  xy_goal_tolerance: 0.003
  yaw_goal_tolerance: 0.01
  trans_stopped_velocity: 0.0005
  rot_stopped_velocity: 0.0005
```

### test_experiments.yaml — DWB (참고, 이 실험에서는 미사용)

```yaml
DWB:
  sim_time: 0.3
  GoalAlign: 활성화
  costmap resolution: 0.01
```

### move_to_start

- position tolerance: 5mm
- yaw tolerance: 없음 (position만 체크) ← **이 시점에서 yaw 미체크**
- timeout: 10s
- 포지셔닝 실패 시: 3회 재시도 후 SKIP_POSITIONING

### 데이터 노트

- 양의 heading(+3°,+4°,+5°)에서 yaw 포지셔닝 부족 → base_link 시작점 ±33mm 편차
- filtered 버전: MPPI_20260416_074529_simulation_filtered/ (17 trials 제거, 127 유효)

### 결과

- 원본: 75/144 SUCCESS (52.1%), 28 TIMEOUT_SETTLE, 41 TIMEOUT_TOTAL
- filtered: 66/127 SUCCESS (52.0%), 25 TIMEOUT_SETTLE, 36 TIMEOUT_TOTAL
