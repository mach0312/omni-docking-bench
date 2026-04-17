# 실험 결과 해석 가이드

## 디렉토리 구조

```
results/
├── kinematic_20260414_111026/     ← 실험 1회 분량
│   ├── summary.csv                ← trial별 요약 (시작/종료 위치, 결과)
│   ├── timeseries/
│   │   ├── trial_001.csv          ← 50Hz 시계열 (pose + cmd_vel + joint)
│   │   └── ...
│   ├── metrics.csv                ← trial별 7개 평가 지표
│   ├── report.txt                 ← 통계 요약
│   └── plots/                     ← 시각화 PNG
├── mppi_20260415_.../
├── dwb_20260415_.../
└── comparison_20260415_.../       ← 컨트롤러 비교 결과
```

---

## 7개 평가 지표

### 1. Settling time (sec)

시행 시작부터 목표 3mm 이내 최초 도달까지의 시간.

$$t_s = \min\{t \mid \sqrt{x(t)^2 + y(t)^2} < 0.003\} - t_0$$

- **좋은 값**: 작을수록 좋음 (빠른 수렴)
- **논문 활용**: 컨트롤러 반응 속도 비교의 핵심 지표

### 2. Steady-state error (mm)

시행 최종 1초간 목표까지 거리의 평균.

$$e_{ss} = \frac{1}{N_{1s}} \sum_{t \in [t_{end}-1, t_{end}]} \sqrt{x(t)^2 + y(t)^2}$$

- **좋은 값**: 0에 가까울수록 정밀
- **논문 활용**: 정밀 도킹 능력의 직접 지표

### 3. Max overshoot (mm)

목표를 최초 통과(3mm 이내 진입)한 이후 다시 멀어진 최대 거리.

$$O_{\max} = \max_{t > t_s} \sqrt{x(t)^2 + y(t)^2}$$

- **좋은 값**: 작을수록 좋음 (0이면 오버슈트 없이 수렴)
- **논문 활용**: 제어 안정성 평가. 진동/불안정 여부 판단

### 4. Cross-track error RMS (mm)

출발점→목표점 직선 경로 대비 횡방향 이탈의 RMS.

출발점 $\mathbf{p}_0 = (x_0, y_0)$, 목표점 $\mathbf{p}^* = (x^*, y^*)$, 직선 방향 벡터 $\mathbf{d} = \mathbf{p}^* - \mathbf{p}_0$일 때, 시각 $t$에서의 횡방향 거리:

$$e_{\perp}(t) = \frac{|(p_x(t) - x_0) \cdot d_y - (p_y(t) - y_0) \cdot d_x|}{|\mathbf{d}|}$$

$$\text{CTE}_{rms} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} e_{\perp}(t_i)^2}$$

- **좋은 값**: 작을수록 직선 경로에 가까움
- **논문 활용**: 경로 효율성. 기구학 기반은 단일 직선 수렴이므로 낮을 것으로 예상

### 5. Heading error (deg)

시행 종료 시점의 목표 대비 헤딩 오차.

$$e_{\psi} = |\text{normalize}(\psi(t_{end}) - \psi^*)|$$

- **좋은 값**: 작을수록 좋음
- **논문 활용**: 도킹 방향 정밀도

### 6. Control effort

속도 명령의 변화량(가속) 누적. 제어 입력의 부드러움을 정량화.

$$E_u = \sum_{i=1}^{N-1} \sqrt{(v_{x,i+1} - v_{x,i})^2 + (v_{y,i+1} - v_{y,i})^2}$$

- **좋은 값**: 작을수록 부드러운 제어 (낮은 에너지 소모)
- **논문 활용**: MPPI/DWB는 샘플링 기반이라 명령 진동이 클 수 있음

### 7. Steering change (rad)

**조향(steering) 조인트만** 각도 변화량 누적. wheel 조인트는 제외. 캐스터 정렬 교란을 정량화.

$$S = \sum_{i=1}^{N-1} \sum_{j \in \{\text{front\_steer}, \text{rear\_steer}\}} |\theta_{j,i+1} - \theta_{j,i}|$$

- **좋은 값**: 작을수록 좋음 (조향 적으면 캐스터 정렬 유지)
- **논문 활용**: **핵심 차별화 지표**. 조향이 많으면 캐스터 정렬이 흐트러져 수렴 방해. 기구학 기반은 단일 경로 수렴으로 조향 최소화

---

## 생성 그래프 — 단일 실험 (plot_single.py)

### convergence.png — 수렴 곡선

시간(x) vs 목표까지 거리(y)를 trial별로 오버레이.

$$d(t) = \sqrt{(x(t) - x^*)^2 + (y(t) - y^*)^2} \times 1000 \quad [\text{mm}]$$

- **색상**: 8방위별 구분
- **선 스타일**: 헤딩 오프셋별 구분 (실선=0°, 점선=-20°, 파선=+20°)
- **빨간 점선**: 3mm 성공 기준선
- **주황 점선**: 5mm settle 기준선
- **읽는 법**: 곡선이 빠르게 3mm 아래로 내려가면 수렴 성능 우수. 곡선 간 산포가 작으면 방향 강건성 우수.

### direction_bars.png — 방향별 지표 bar chart

8방위 각각에 대해 7개 지표의 평균±표준편차.

- **읽는 법**: bar 높이가 균일하면 방향 의존성 낮음. 특정 방위에서 bar가 높으면 해당 방향이 약점.

### polar.png — 방향별 극좌표 그래프

8방위의 settling time과 SSE를 극좌표로 표시.

- **읽는 법**: 원에 가까우면 방향 의존성 없음 (등방성). 한쪽이 돌출되면 해당 방향에서 성능 저하.

### heading_bars.png — 헤딩별 지표 bar chart

(헤딩 2종 이상일 때만 생성)

헤딩 오프셋(0°, -20°, +20° 등)별 7개 지표의 평균±표준편차.

- **읽는 법**: 헤딩에 따른 성능 차이를 직접 비교. bar 차이가 크면 초기 헤딩에 민감.

### heatmap_settling.png — 방위×헤딩 히트맵

(헤딩 2종 이상일 때만 생성)

행=8방위, 열=헤딩 오프셋, 셀 값=settling time 평균.

- **색상**: 녹색(빠름) → 적색(느림)
- **읽는 법**: 적색 셀이 집중된 방위-헤딩 조합이 최악의 조건. 전체가 녹색이면 강건함.

### steering_vs_settling.png — 조향-수렴 상관 scatter

X축=steering change (rad), Y축=settling time (s). 점 1개 = trial 1회.

$$r = \text{corr}(S, t_s)$$

- **추세선**: 선형 회귀 (기울기, 상관계수 r 표시)
- **읽는 법**: 양의 상관(r > 0)이면 "조향이 많을수록 수렴이 느림"을 의미. r이 1에 가까울수록 강한 상관.
- **논문 활용**: 리더님 요청 — "조향이 많으면 캐스터 정렬이 흐트러져 수렴에 영향" 주장의 정량적 근거. 3개 컨트롤러 합산 시 MPPI/DWB는 우상단(조향 많고 느림), Kinematic은 좌하단(조향 적고 빠름)에 위치할 것으로 예상.

---

## 생성 그래프 — 컨트롤러 비교 (plot_compare.py)

### boxplots.png — 지표별 box plot

7개 지표 각각에 대해 컨트롤러별 box plot (분포 비교).

- **색상**: 빨강=Kinematic(제안), 파랑=MPPI, 초록=DWB
- **해치 패턴**: 흑백 인쇄 대비 (빗금 구분)
- **읽는 법**: 중앙값(선), 사분위(상자), 이상치(점). 상자가 낮을수록 해당 지표에서 우수.

### radar.png — 정규화 radar chart

7개 지표를 0~1로 정규화하여 컨트롤러별 오버레이.

정규화: 각 지표에서 최솟값=1(최고), 최댓값=0(최저).

$$\hat{m}_k = 1 - \frac{m_k - m_{k,\min}}{m_{k,\max} - m_{k,\min}}$$

- **읽는 법**: 면적이 클수록 전반적 성능 우수. 한 축만 돌출되면 해당 지표에서만 강점.

### convergence_compare.png — 방향별 수렴 곡선 비교

대표 방향(0°, 90°, 180°, 270°)에서 컨트롤러별 수렴 곡선 오버레이.

- **읽는 법**: 같은 방향에서 어떤 컨트롤러가 빠르게 수렴하는지 직접 비교.

### success_rate.png — 성공률 bar

컨트롤러별 전체 성공률(%).

- **읽는 법**: 100%에 가까울수록 안정적. 실패가 있으면 해당 컨트롤러의 한계 조건 존재.

### heading_robustness.png — 헤딩 강건성

(헤딩 2종 이상일 때만 생성)

헤딩 오프셋별 settling time을 컨트롤러별로 비교.

- **읽는 법**: 헤딩 변화에 따른 settling time 증가가 적으면 강건. Kinematic이 MPPI/DWB보다 헤딩 변화에 둔감하면 강건성 우위.

### steering_vs_settling.png — 조향-수렴 scatter (비교)

3개 컨트롤러의 데이터를 색상별로 합산한 scatter.

- **읽는 법**: 컨트롤러별 군집 위치가 핵심. Kinematic=좌하단(조향↓수렴↑), MPPI/DWB=우상단(조향↑수렴↓) 패턴 예상.

### comparison.csv — 비교 요약 테이블

컨트롤러별 7개 지표의 mean, std, min, max. 논문 Table 작성에 직접 활용.

---

## 사용법

```bash
# 단일 실험 (실험 완료 시 자동 실행)
python3 scripts/plot_single.py results/kinematic_20260414_111026

# 컨트롤러 비교
python3 scripts/plot_compare.py \
  results/kinematic_... \
  results/mppi_... \
  results/dwb_...
```
