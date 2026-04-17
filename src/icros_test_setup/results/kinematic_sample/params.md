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

### controller 파라미터 (pgv_controller)
- max_lin_vel: 0.1 m/s
- max_ang_vel: 0.05 rad/s
- fine_xy_tolerance: 0.003m
- fine_yaw_tolerance_deg: 2.0°
- sensor_y_offset: 0.51m
