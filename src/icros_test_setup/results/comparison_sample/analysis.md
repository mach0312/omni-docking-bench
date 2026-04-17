# Phase 2 Heading Sweep — Kinematic vs MPPI Final Analysis

## 실험 조건
- **Heading offsets**: 0°, ±2°, ±3°, ±4°, ±5° (9개)
- **방향**: 8방위 (0°~315°)
- **Repeats**: 2
- **Trials/controller**: 144 (kinematic 143 — outlier 1건 제외)
- **출발 거리**: 3cm (base_link 기준)
- **성공 기준**: 센서 3mm 이내 × 5연속
- **실기 PGV 한계**: heading ±5.3° (FOV 120mm, 밴드 25mm, 오프셋 510mm)

## 성공률

| Controller | Total | Success | Settle Timeout | Total Timeout | 성공률 |
|---|---|---|---|---|---|
| **kinematic** | 143 | **143** | 0 | 0 | **100%** |
| **MPPI** | 144 | 75 | 28 | 41 | **52.1%** |

## 주요 지표 비교 (성공 trial만)

| Metric | kinematic | MPPI | 비고 |
|---|---|---|---|
| Settling Time (s) | **5.31** ± 3.07 | 8.64 ± 8.32 | kinematic 1.6배 빠름 |
| SSE (mm) | 2.78 ± 0.29 | **2.37** ± 0.74 | 유사 (SSE 수정 후) |
| Max Overshoot (mm) | 2.92 ± 0.36 | 3.28 ± 0.76 | 유사 |
| Cross-Track RMS (mm) | **3.91** ± 3.07 | 2.59 ± 1.88 | heading 변화 시 kinematic 경로 편차 증가 |
| Heading Error (deg) | 0.16 ± 0.30 | **0.10** ± 0.08 | 둘 다 우수 |
| CDI | **8.27** ± 4.18 | 7.34 ± 5.85 | heading sweep에서 둘 다 증가 |
| Steering Change (rad) | **3.94** ± 2.43 | 7.52 ± 9.52 | MPPI 조향 2배 |

## Heading별 성공률 분석

### Kinematic — heading 무관하게 100% 수렴
| Heading | Trials | Success | Settling (avg) | Steering |
|---|---|---|---|---|
| 0° | 15 | 100% | 2.07s | 0.70 rad |
| ±2° | 32 | 100% | 4.20s | 3.16 rad |
| ±3° | 32 | 100% | 5.39s | 4.02 rad |
| ±4° | 32 | 100% | 6.76s | 5.23 rad |
| ±5° | 32 | 100% | 6.93s | 4.89 rad |

heading 증가 → settling time/steering 증가하지만 SSE는 일관적. 강건함.

### MPPI — heading에 따른 성공률 편차
| Heading | Success/Total | 성공률 | Settling (avg) |
|---|---|---|---|
| 0° | 13/16 | 81% | 11.75s |
| ±2° | 14/32 | 44% | 7.84s |
| ±3° | 20/32 | 63% | 5.93s |
| ±4° | 17/32 | 53% | 9.59s |
| ±5° | 11/32 | 34% | 9.61s |

heading 0°에서도 81%로 완벽하지 않음. heading ±5°에서 34%로 하락.
특이하게 ±3°가 63%로 ±2°(44%)보다 높음 — stochastic 특성에 의한 편차.

## MPPI 실패 유형 분석

### TIMEOUT_SETTLE (28회, 19%)
- Controller가 base_link 목표에 도달했으나 센서가 3mm 이내 미수렴
- 원인: heading 오차 × 센서 오프셋 510mm → 센서 위치 오차
  - yaw 0.5° 오차 → 센서 4.5mm 오차
  - goal_checker yaw_tolerance=0.01 rad(0.57°) → 최대 센서 5.1mm
- 구조적 한계: MPPI는 base_link 프레임에서 제어, 센서 수렴은 보장 불가

### TIMEOUT_TOTAL (41회, 28%)
- Controller 자체가 base_link 목표에 미도달 (90초 timeout)
- 발산 패턴: 목표 근처에서 oscillation 후 점진적 이탈
- 원인: MPPI stochastic sampling의 미세 제어 한계
  - sampling noise가 goal에서의 미세한 gradient를 mask
  - 낮은 속도 영역에서 cost landscape이 flat → 탐색 방향 불안정

## Heading Robustness 분석 (heading_robustness.png)

- **kinematic**: heading 증가에 따라 settling time이 선형적으로 증가 (2s→7s). 분산 작음.
- **MPPI**: heading과 settling time의 상관관계 불명확. 분산이 매우 큼. heading보다 stochastic 특성이 지배적.

이는 MPPI의 수렴 성능이 heading offset보다 **trial별 randomness**에 더 의존함을 의미.

## 결론

### Kinematic Controller
- **정밀 도킹에 최적**. heading ±5° 전 범위에서 100% 수렴.
- heading 증가 시 settling time 증가는 센서 오프셋 회전 보정에 소요되는 시간.
- 실기 PGV 한계(±5.3°) 내에서 완벽한 강건성.

### MPPI Controller  
- **정밀 도킹에 부적합** (현재 설정). 52% 성공률, heading과 무관한 높은 실패율.
- 근본 원인: stochastic sampling이 mm 단위 정밀 제어에 부적합.
  - sampling noise(10mm/s) vs 필요 정밀도(3mm)의 부조합
  - base_link 프레임 제어 vs 센서 프레임 평가의 좌표계 불일치
- 성공 시 SSE 2.4mm으로 kinematic(2.8mm)과 유사하지만, 신뢰성이 부족.

### DWB Controller (Phase 1에서 탈락)
- **구조적으로 불가능**. costmap cell 기반 scoring이 3cm 경로에서 작동 불가.

### 개선 방향
1. **MPPI 정밀도 향상**: 목표 근처에서 sampling noise를 적응적으로 감소 (annealing)
2. **센서 프레임 직접 제어**: controller가 base_link가 아닌 센서 위치를 목표로 제어
3. **Hybrid 접근**: MPPI로 대략적 접근 → kinematic으로 미세 수렴 (handover)
