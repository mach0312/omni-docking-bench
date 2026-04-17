# icros_test_setup

전방향 AMR(DSD-bot)에서 정밀 도킹 컨트롤러를 비교하기 위한 **ICROS 2026 실험 패키지**.
PGV100-F200A-R4 마커 기반 위치 센서를 사용하여 **3 cm 근거리에서 3 mm 정밀도 수렴** 성능을 비교한다.

> 본 패키지는 [`omni-docking-bench`](../../README.md) 저장소의 일부이며,
> 논문의 실험 자동화 / 지표 계산 / 시각화 파이프라인을 제공합니다.

---

## 비교 컨트롤러

| 컨트롤러 | 유형 | 역할 |
|----------|------|------|
| **Kinematic** | PGV 직접 피드백 제어 (`pgv_guided_controller`) | **제안 방법** — 기구학 기반 수렴 제어 |
| **MPPI** | 샘플링 기반 MPC (Nav2) | 기존 방법 — 최신 최적 제어 |
| **DWB** | 동적 윈도우 (Nav2) | 기존 방법 — 전통 반응형 |

## 실험 프로토콜 (기본값)

- 출발 거리: 3 cm (목표점 기준)
- 출발 방향: 8방위 (45도 간격)
- 헤딩 오프셋: launch arg `heading_offsets_deg` (기본 `[0.0]`, 예: `[0.0,-3.0,3.0]`)
- 반복: launch arg `repeats` (기본 3)
- 총 시행: 컨트롤러 × 8방위 × heading × repeats

예) 헤딩 1종 × 3반복 → 컨트롤러 당 24회, 총 72회

---

## 패키지 구조

```
icros_test_setup/
├── icros_test_setup/
│   ├── experiment_runner_node.py   # 시행 자동화 + 시계열 기록
│   ├── experiment_analyzer.py      # 7개 지표 계산 + report 생성
│   ├── plot_single.py              # 단일 실험 시각화
│   └── dummy_pgv_node.py           # PGV 시뮬레이터 (마커 기반 map→odom TF 발행)
├── scripts/
│   ├── plot_compare.py             # 컨트롤러 비교 시각화
│   └── run_plot_compare.sh
├── launch/
│   ├── controller_only.launch.py   # Nav2 controller_server (MPPI/DWB 전용)
│   ├── experiment_kinematic.launch.py
│   ├── experiment_mppi.launch.py
│   └── experiment_dwb.launch.py
├── config/
│   ├── experiment_scenarios.yaml
│   └── test_experiments.yaml       # Nav2 controller_server 파라미터
├── docs/
│   ├── dummy_pgv_node.md
│   └── pgv_roi_analysis.md
└── results/                        # 실험 결과 저장 (sample 만 git 추적)
```

---

## 노드 설명

### experiment_runner_node

시행 자동화: 시작점 이동 → 안정화 → 시행 실행 → CSV/시계열 기록 → analyzer 자동 실행.

**시행 흐름:**
1. `pgv_controller` 로 시작점 (x, y, yaw) 이동, 5 mm 이내 0.5초 안정화 대기
2. 1초 추가 안정화
3. 실제 시작 위치 기록
4. 설정된 컨트롤러로 goal 까지 이동
   - **kinematic**: `/pgv_controller/goal` 직접 발행 (FollowPath 미사용)
   - **MPPI/DWB**: Nav2 `FollowPath` action 호출
5. PGV 실측 기반 종료 판정
6. summary CSV + 시계열 CSV 기록
7. 전체 완료 후 analyzer + plot 자동 실행

**종료 판정:**
- 성공: 연속 5샘플(0.1초) 3 mm 이내
- 수렴 실패: 5 mm 이내 체류 누적 60초 후에도 3 mm 미도달
- 전체 타임아웃: 90초 절대 제한

**주요 파라미터:**
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `controller_name` | `mppi` | CSV 기록용 컨트롤러 이름 |
| `pose_topic` | `/pgv/raw_pose` | PGV 센서 토픽 |
| `results_dir` | `~/omni_docking_bench_ws/src/icros_test_setup/results` | 결과 저장 경로 |
| `repeats` | `3` | 방향당 반복 횟수 |
| `heading_offsets_deg` | `[0.0]` | 헤딩 오프셋 목록 (deg) |
| `success_threshold` | `0.003` | 성공 판정 거리 [m] |
| `success_samples` | `5` | 성공에 필요한 연속 샘플 수 |
| `settle_threshold` | `0.005` | 수렴 영역 거리 [m] |
| `settle_timeout` | `60.0` | 수렴 영역 누적 체류 제한 [s] |
| `total_timeout` | `90.0` | 전체 절대 시간 제한 [s] |

### dummy_pgv_node

실제 PGV 하드웨어를 대신하여 PGV `PoseStamped` 를 publish 하고
`map→odom` TF 를 발행하는 시뮬레이터. `ros2_control` 의
`odom→base_link` TF 와 결합하여 TF 체인이 닫히도록 구성된다.

---

## 실험 실행

모든 실험 launch 는 CLI 에서 다음 arg 를 override 할 수 있습니다.

| 인자 | 예시 | 설명 |
|------|------|------|
| `heading_offsets_deg` | `"[0.0,-3.0,3.0]"` | 헤딩 오프셋 list (deg) |
| `repeats` | `3` | 방향 · 헤딩당 반복 횟수 |
| `results_dir` | `/tmp/my_run` | 출력 CSV 경로 (빈 값 = 기본) |

### Kinematic (제안 방법)

```bash
ros2 launch icros_test_setup experiment_kinematic.launch.py
# 헤딩 1종, 3반복 = 24회
ros2 launch icros_test_setup experiment_kinematic.launch.py \
  heading_offsets_deg:='[0.0]' repeats:=3
```

### MPPI / DWB

```bash
# 터미널 1: Nav2 controller server (MPPI 또는 DWB 파라미터 사용)
ros2 launch icros_test_setup controller_only.launch.py \
  params_file:=$(ros2 pkg prefix icros_test_setup)/share/icros_test_setup/config/test_experiments.yaml

# 터미널 2: 실험
ros2 launch icros_test_setup experiment_mppi.launch.py
# 혹은
ros2 launch icros_test_setup experiment_dwb.launch.py
```

---

## 결과 구조

실험 완료 시 자동 생성:

```
results/kinematic_20260414_111026/
├── summary.csv                  # trial별 요약
├── timeseries/
│   ├── trial_001.csv            # 50 Hz 시계열 (t, x, y, yaw, vx, vy, wz, j0~j3)
│   └── ...
├── metrics.csv                  # trial별 7개 평가 지표
├── report.txt                   # 통계 요약
└── plots/
    ├── convergence.png
    ├── direction_bars.png
    ├── polar.png
    ├── heading_bars.png
    ├── heatmap_settling.png
    └── steering_vs_settling.png
```

## 데이터 분석

### 단일 실험 분석 (자동 실행)

```bash
python3 -m icros_test_setup.experiment_analyzer results/kinematic_20260414_111026
```

### 컨트롤러 비교

```bash
python3 scripts/plot_compare.py \
  results/kinematic_20260414_... \
  results/mppi_20260415_... \
  results/dwb_20260415_...
```

출력: `results/comparison_<timestamp>/{boxplots,radar,convergence_compare,success_rate,heading_robustness,steering_vs_settling}.png` + `comparison.csv`.

## 평가 지표 (7개)

| 지표 | 단위 | 정의 |
|------|------|------|
| Settling time | sec | 출발 → 3 mm 최초 도달 시간 |
| Steady-state error | mm | 최종 1초간 위치 오차 평균 |
| Max overshoot | mm | 목표 통과 후 최대 이탈 |
| Cross-track error (RMS) | mm | 출발→목표 직선 대비 횡방향 오차 |
| Heading error (final) | deg | 최종 헤딩 오차 |
| Control effort | m/s | 속도 변화량 누적 |
| Steering change | rad | 조향 변화량 누적 (캐스터 정렬 영향 정량화) |

상세 해석은 `results/README.md` 참고.

---

## TF 구조

```
map    --(dummy_pgv_node)-->    odom    --(ros2_control)-->    base_link
```

- `dummy_pgv_node` 가 마커 기반 pose 를 이용하여 `map→odom` 을 발행한다.
- `ros2_control` (double_steering_drive_controller) 이 `odom→base_link` 를 발행한다.
- 별도의 robot pose publisher 는 불필요.

---

## 의존성

| 패키지 | 출처 | 용도 |
|--------|------|------|
| `pgv_guided_controller` | 같은 모노 레포 | Kinematic 제안 제어기 |
| `dsd_bot_description` | 같은 모노 레포 | URDF / xacro |
| `dsd_control_demo` | 같은 모노 레포 | 하드웨어 시스템 인터페이스 / bringup |
| `nav2_bringup` + `nav2_controller` | Nav2 | MPPI / DWB controller_server |
| `matplotlib` | pip | 시각화 |

## 빌드

루트 워크스페이스에서:

```bash
cd ~/omni_docking_bench_ws
colcon build --packages-select icros_test_setup
source install/setup.bash
```

검증 환경: ROS 2 Jazzy (Ubuntu 24.04)

## 라이선스

Apache-2.0
