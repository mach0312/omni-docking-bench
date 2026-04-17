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

### controller 파라미터 (MPPI)
- time_steps: 20
- model_dt: 0.05
- batch_size: 2000
- vx_std: 0.01
- vy_std: 0.01
- wz_std: 0.02
- vx_max: 0.1
- vy_max: 0.1
- wz_max: 0.2
- iteration_count: 2
- temperature: 0.3
- motion_model: Omni

### MPPI critics
- ConstraintCritic (w=4.0)
- CostCritic (w=3.81)
- GoalCritic (w=20.0, threshold=0.5)
- GoalAngleCritic (w=10.0, threshold=0.2)
- VelocityDeadbandCritic (w=10.0, deadband=[0.001, 0.001, 0.001])

### goal_checker
- xy_goal_tolerance: 0.003m
- yaw_goal_tolerance: 0.01 rad
- trans_stopped_velocity: 0.0005 m/s
- rot_stopped_velocity: 0.0005 rad/s
