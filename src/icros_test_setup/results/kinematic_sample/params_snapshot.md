# Parameter Snapshot — kinematic Phase 2
## 실험: kinematic_20260415_194249_simulation
## 단계: Phase 2 Heading Sweep (heading 0°,±2°,±3°,±4°,±5°, 2 repeats, 144 trials)

### experiment_runner 파라미터
- controller_name: kinematic
- heading_offsets_deg: [0.0, -2.0, 2.0, -3.0, 3.0, -4.0, 4.0, -5.0, 5.0]
- repeats: 2
- success_threshold: 0.003m
- success_samples: 5
- settle_timeout: 60s
- total_timeout: 90s
- sensor_offset: (0.0, 0.51)
- 출발점: base_link 기준 (bl_goal 역산 적용)
- 모니터링: _monitor_trial (센서 수렴만 감시)

### controller 파라미터
- pgv_controller (pgv_controller.params.yaml) — Phase 1과 동일

### move_to_start
- position tolerance: 5mm
- yaw tolerance: 없음 (position만 체크)
- timeout: 10s
- 포지셔닝 실패 시: 3회 재시도 후 SKIP_POSITIONING

### 데이터 노트
- trial 1 제거 (포지셔닝 실패 — 380mm 떨어진 곳에서 시작)
- 유효 trials: 143/144
- heading 부동소수점 수정: 3.0000000000000004 → 3.0

### 결과: 143/143 SUCCESS (100%)
