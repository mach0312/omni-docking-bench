# Precision Docking Controller Comparison - Experiment Log

## 실험 목적
옴니 AMR에서 kinematic / MPPI / DWB 3개 컨트롤러의 정밀 도킹 수렴성 비교.
8방위 동일 base_link 출발점에서 다양한 heading offset으로 테스트.

---

## 실험 환경
- 로봇: AMR V2 (Double Steering Drive, wheel_base=1.2215m, wheel_radius=0.125m)
- 센서: dummy_pgv_node (PGV 시뮬레이션, offset_y=0.51m, noise=0.2mm)
- 컨트롤러 주기: 50Hz
- 최대 속도: 0.1 m/s (lin), 0.1~0.2 rad/s (ang)
- Goal checker: xy=3mm, yaw=0.05rad, stopped_vel=0.5mm/s

### 물리적 제약 (실제 PGV 하드웨어)
- 센서 FOV: 120mm x 80mm (읽기 거리 100mm)
- 코드 밴드 폭: **25mm** (PGV-CA25, Width:1)
- 센서 오프셋: 510mm → heading 회전 시 센서 횡이동 = 510mm × sin(θ)
- ROI 판단: 코드(25mm)가 FOV(120mm) 안에 완전 포함 필요
- 최대 허용 횡이동: (120 - 25) / 2 = **47.5mm**
- 최대 heading: arcsin(47.5 / 510) = **±5.3°**
- 상세 분석: [docs/pgv_roi_analysis.md](docs/pgv_roi_analysis.md)
- dummy_pgv는 밴드 폭 미시뮬레이션 (무한 FOV)

---

## 실험 설계

### 공통 조건
- 출발 거리: 3cm (base_link 기준, 목표로부터)
- base_link 목표: (0.0, -0.51) — 센서 목표 (0,0) + offset 역변환
- 같은 방향의 모든 heading에서 base_link 출발점 동일 (센서 목표만 heading에 따라 역산)
- success_threshold: 3mm (센서 기준)
- total_timeout: 90s

### Phase 1: Baseline (heading=0° only)
- **목적**: 3개 컨트롤러 기본 동작 검증
- 8방향 × 1 heading(0°) × 3 repeats = **24 trials/controller**
- post_controller_timeout: 5s (MPPI/DWB)

### Phase 2: Heading Sweep (실기 유효 범위)
- **목적**: heading 변화에 대한 수렴 한계 탐색
- Headings: 0°, ±2°, ±3°, ±4°, ±5° (총 9개, 실기 한계 ±5.3° 이내)
- 8방향 × 9 headings × 2 repeats = **144 trials/controller**
- ±5°는 FOV 여유 3.1mm으로 경계 조건

### Phase 3: 심층 분석 (Phase 2 결과에 따라 설계)
- 실패 구간 집중 테스트
- 알고리즘별 실패 원인 분석

---

## 변경 이력

### 2026-04-15: 초기 설정

#### 1. base_link 기준 출발점 통일 (experiment_runner_node.py)
- **변경**: trial 생성 시 base_link 목표 위치를 먼저 계산하고, heading에 따라 센서 목표를 역산
- **이유**: 기존에는 센서 좌표 기준 출발점이 고정되어 heading 변경 시 base_link가 ±174mm 이동
- **수식**: `sensor_start = bl_start + R(heading) × offset`

#### 2. Action 기반 모니터링 분리 (experiment_runner_node.py)
- **변경**: MPPI/DWB용 `_monitor_trial_action()` 추가
- **이유**: controller 완료 시점과 센서 수렴을 분리 판정
- **흐름**: Phase1(CTRL) controller 완료 대기 → Phase2(SETTLE) 센서 수렴 판정 (post_controller_timeout)

#### 3. base_link trajectory 로깅 (experiment_runner_node.py)
- **변경**: timeseries에 bl_x, bl_y, bl_yaw 컬럼 추가 (map→base_link TF)
- **이유**: 센서 좌표와 base_link 좌표 모두 기록하여 trajectory 시각화

#### 4. trajectory plot 추가 (plot_single.py)
- **변경**: 방향별 subplot, heading=선스타일, 통일 축 범위, 로봇 좌표계(Y→가로, X→세로)
- **이유**: 다회 실험 시 trial 간 구분 및 비교 용이

#### 5. sleep 축소 (experiment_runner_node.py)
- **변경**: 초기 대기 3s→1s, 안정화 1s→0.3s, trial 간격 0.5s→0.2s, positioning timeout 15s→10s
- **이유**: 실험 소요 시간 단축 (timeout은 유지)

#### 6. MPPI VelocityDeadbandCritic 수정 (test_experiments.yaml)
- **변경**: deadband [0.005,0.005,0.005] → [0.001,0.001,0.001], weight 35→10
- **이유**: deadband 5mm/s가 3mm 목표 수렴을 위한 미세 속도를 억제.
  Phase 1 초기 실행에서 8/8 trial TIMEOUT (센서 5~8mm에서 정체).
  controller가 목표 근처에서 0 또는 deadband 이상만 출력 → 3mm 이내 수렴 불가.
- **분석**: GoalChecker의 stopped_velocity=0.5mm/s와 deadband=5mm/s의 갭이 핵심.
  MPPI sampling이 deadband 경계에서 oscillation.
- **결과**: deadband만 줄여도 미수렴 지속 → 추가 원인 분석 필요

#### 7. MPPI 추가 수정 (test_experiments.yaml) — 2차 시도
- **변경 1**: `use_sim_time: True → False` — mock hardware는 wall clock 사용, TF timestamp 불일치 방지
- **변경 2**: sampling noise 축소 `vx_std 0.05→0.01, vy_std 0.05→0.01, wz_std 0.1→0.02`
  - 기존 noise 50mm/s >> 필요 정밀도 3mm/s. 미세 제어를 위해 sampling 해상도 향상
- **변경 3**: GoalCritic weight 5→20, threshold 1.4→0.5 / GoalAngleCritic weight 3→10, threshold 0.5→0.2
  - 3cm 근거리에서 GoalCritic이 지배적으로 작용하도록. heading 정밀도도 강화.
  - heading 오차 1°에서 센서 8.9mm 오차 (offset 510mm × sin(1°))
- **근거**: MPPI는 stochastic sampling 기반이라 매크로 경로 추종에 최적화. 미세 도킹은 sampling noise와 critic weight 조정이 핵심.
- **결과**: 여전히 TIMEOUT → PathAlignCritic/PathFollowCritic이 GoalCritic과 충돌 확인

#### 8. MPPI critic 단순화 + yaw tolerance 조임 — 4차 시도
- **변경 1**: PathAlignCritic, PathFollowCritic, TwirlingCritic 비활성화
  - 3cm 경로에서 path-following critic들이 goal-seeking과 충돌하여 수렴 방해
  - GoalCritic(w=20) + GoalAngleCritic(w=10) 중심으로 재구성
- **변경 2**: iteration_count 1→2 (sampling 품질 개선)
- **결과**: 11/24 SUCCESS (46%), 5 TIMEOUT_SETTLE, 8 TIMEOUT_TOTAL
  - SETTLE 실패: controller 완료 시 sensor 4~6.6mm → heading 오차 × offset 문제
  - TOTAL 실패: rep 3 후반에서 발산 (246mm까지) → controller 상태 오염 추정
- **변경 3**: yaw_goal_tolerance 0.05→0.01 rad (2.86°→0.57°)
  - sensor 3mm 기준 필요 yaw 정밀도: arcsin(3/510)=0.34° → 여유 포함 0.57°

#### 9. SSE 계산 방식 수정 (experiment_analyzer.py)
- **변경**: "마지막 1초 평균" → "최초 3mm 도달 이후 전체 평균"
- **이유**: 기존 방식에서 빠르게 수렴하는 kinematic의 SSE가 과대 평가 (7.7mm→2.8mm)
  trial이 짧으면 접근 구간이 SSE 윈도우에 포함되는 문제

---

## Phase 1 Baseline 결과

### Kinematic (results/kinematic_20260415_164622)
- **24/24 SUCCESS (100%)**
- settling: avg 2.03s (min 1.98, max 2.12)
- SSE: avg 7.7mm
- heading error: avg 0.02°
- 방향간 편차 거의 없음. 완벽한 수렴.

### MPPI (results/MPPI_20260415_184135)
- **10/24 SUCCESS (41.7%)**, 5 TIMEOUT_SETTLE, 9 TIMEOUT_TOTAL
- 성공 trial만: settling avg 8.1s (min 2.9, max 34.2), SSE avg 2.0mm, heading_error avg 0.10°
- 실패 유형 분석:
  - **TIMEOUT_SETTLE (5회)**: controller 완료 시 sensor 3~7mm, heading 오차 × offset로 센서 미수렴
  - **TIMEOUT_TOTAL (9회)**: controller 자체 미완료. rep 3(trial 19~24)에서 집중 발생, 최대 206mm 발산
- 발산 원인 추정: 반복 trial에서 MPPI optimizer 상태 오염, 또는 odom drift 누적
- 파라미터 변경 이력: deadband 0.005→0.001, sampling noise 5x 축소, PathAlign/Follow 비활성화, GoalCritic w=20, yaw_tolerance 0.05→0.01 rad

### DWB (7/7 TIMEOUT_TOTAL — 완전 발산)
- **0/24 SUCCESS (0%)**
- 모든 trial에서 목표로부터 0.7~1.0m 발산
- **근본 원인**: DWB가 `vel=(0, 0, -0.1)` — 순수 회전만 출력
  - StandardTrajectoryGenerator가 omni 경로를 제대로 scoring하지 못함
  - GoalDist critic이 회전 trajectory에 높은 점수 부여 (rotation이 goal에 가까워지는 것으로 평가)
  - 센서 오프셋 510mm에서 회전 → 센서가 원을 그리며 발산
- **결론**: DWB는 omni 로봇의 정밀 도킹에 구조적으로 부적합
  - DWB의 trajectory generator/scoring이 differential drive 모델 기반
  - MPPI의 `motion_model: "Omni"` 같은 명시적 omni 지원 없음
- DWB 파라미터 수정 이력: sim_time 0.3→1.5, samples 10→15 (효과 없음)

#### 10. DWB 수정 — 소스 코드 기반 분석 후 (test_experiments.yaml)
- **근본 원인** (Nav2 소스 확인):
  - GoalDist critic은 trajectory 끝점의 costmap cell Manhattan 거리로 scoring
  - costmap resolution 0.05m에서 3cm path는 1 cell 이내 → 모든 trajectory가 동일 점수
  - sim_time=0.3에서: linear trajectory 끝점과 rotation trajectory 끝점이 같은 cell → 구분 불가
  - sim_time=1.5에서: linear trajectory가 15cm overshoot → rotation보다 나쁜 점수
  - 결과: 어떤 sim_time이든 rotation이 최적 또는 동률
- **수정 내용**:
  1. GoalAlign critic 활성화 — heading 방향 기반 scoring, cell 해상도 무관 (forward_point_distance=0.001)
  2. costmap resolution 0.05→0.01m — 3cm path가 3 cells로 확장, GoalDist 구분 가능
  3. sim_time 0.3 복원 — 0.3s × 0.1m/s = 3cm = path 길이 (overshoot 방지)
  4. samples 15→10 복원, max_vel_theta 0.05→0.1 복원
- **소스 참조**: `nav2_dwb_controller/dwb_critics/src/goal_dist.cpp` (Last aggregation), `map_grid.cpp` (Manhattan BFS)
- **결과**: GoalAlign + resolution 0.01m 적용 후에도 순수 회전 지속. 추가 원인:
  - 정지 상태에서 acc_lim=1.5, dt=0.02s → 첫 cycle vx_max=0.03m/s
  - sim_time 0.3s에서 trajectory 최대 도달 9mm → GoalDist 1~2 cell 차이 → 불안정한 scoring
- **최종 결론**: DWB는 3cm 정밀 도킹에 **구조적으로 부적합**
  - costmap cell 기반 Manhattan distance scoring이 sub-cell 정밀도에서 작동 불가
  - MPPI의 continuous GoalCritic (Euclidean distance)와 근본적 차이
  - 이후 Phase 2는 kinematic + MPPI만 진행

---

## Phase 1 최종 요약

| Controller | 성공률 | Settling (avg) | SSE (avg) | 비고 |
|---|---|---|---|---|
| **kinematic** | **100%** (24/24) | **2.03s** | 2.76mm | 완벽한 수렴, 일관된 성능 |
| **MPPI** | **42%** (10/24) | 8.06s | 2.10mm | 성공 시 정밀, 발산 위험 있음 |
| **DWB** | **0%** (0/24) | — | — | 순수 회전만 출력, 구조적 한계 |

### Phase 2 진행 방향
- DWB 제외, **kinematic + MPPI**로 heading sweep 진행
- MPPI 발산 문제 (rep 3 후반) 추가 조사 필요

#### 11. 포지셔닝 실패 시 재시도 + skip 처리 (experiment_runner_node.py)
- **변경**: 포지셔닝 실패 시 최대 3회 재시도, 모두 실패 시 `SKIP_POSITIONING`으로 기록하고 건너뜀
- **이유**: Phase 2 kinematic에서 trial 1이 포지셔닝 실패(380mm 떨어진 위치에서 시작)인데 trial을 강행하여 53초 소요. 유효하지 않은 데이터가 유효 데이터와 섞이는 문제.
- **이전 동작**: 포지셔닝 실패해도 `"running anyway"`로 진행

---

## Phase 2 Heading Sweep 결과

### Kinematic (results/kinematic_20260415_194249)
- **143/143 SUCCESS (100%)** (outlier 1건 제외 — 포지셔닝 실패 trial)
- heading ±5° 전 범위에서 완벽한 수렴
- settling: heading 0°→2.1s, ±5°→6.9s (선형 증가)
- SSE: heading 무관하게 일관적 (~2.8mm)

### MPPI (results/MPPI_20260416_074529)
- **75/144 SUCCESS (52.1%)**, 28 TIMEOUT_SETTLE, 41 TIMEOUT_TOTAL
- heading별 성공률: 0°=81%, ±2°=44%, ±3°=63%, ±4°=53%, ±5°=34%
- heading보다 stochastic 특성이 수렴 성능을 지배
- TIMEOUT_SETTLE: base_link 도달 후 센서 미수렴 (yaw 오차 × offset)
- TIMEOUT_TOTAL: controller 자체 발산 (sampling 불안정)

### 상세 분석
- [Phase 2 Comparison Analysis](results/comparison_20260416_092314/analysis.md)
- 비교 plots: [results/comparison_20260416_092314/](results/comparison_20260416_092314/)
