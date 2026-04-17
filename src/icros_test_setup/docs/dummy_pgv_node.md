# Dummy PGV Node

## 개요

실제 PGV(Position Guided Vision) 센서 없이 테스트하기 위한 시뮬레이션 노드.
wheel odometry 데이터를 기반으로 PGV 센서 출력을 모사하여 `/pgv/raw_pose`로 발행한다.

## 데이터 흐름

```
/dsdbot_base_controller/odom (Odometry)
        │
        ▼
  ┌─────────────────┐
  │  Dummy PGV Node  │
  │                   │
  │  1. odom 위치 추출 │
  │  2. init offset   │
  │  3. 센서 오프셋    │
  │  4. 노이즈 추가    │
  └────────┬──────────┘
           │
           ▼
  /pgv/raw_pose (PoseStamped, frame_id='map')
```

## 좌표 변환 수식

### Step 1: odom → map 프레임 매핑

odom 프레임의 base_link 위치 $(x_o, y_o, \theta_o)$를 map 프레임의 로봇 위치 $(x_r, y_r, \theta_r)$로 변환한다.
`init_x`, `init_y`, `init_yaw`는 odom 원점의 map 프레임 내 위치를 정의한다.

$$x_r = x_o + x_{\text{init}}$$

$$y_r = y_o + y_{\text{init}}$$

$$\theta_r = \theta_o + \theta_{\text{init}}$$

### Step 2: base_link → sensor 변환

센서는 base_link에서 body frame 기준 $(d_x, d_y)$ 만큼 오프셋된 위치에 있다.
이를 map 프레임으로 변환하면:

$$x_s = x_r + d_x \cos\theta_r - d_y \sin\theta_r$$

$$y_s = y_r + d_x \sin\theta_r + d_y \cos\theta_r$$

$$\theta_s = \theta_r + \Delta\theta_{\text{yaw}}$$

행렬 표현:

$$\mathbf{p}_s^{\text{map}} = \mathbf{p}_r^{\text{map}} + R(\theta_r) \cdot \mathbf{d}$$

$$R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}, \quad \mathbf{d} = \begin{bmatrix} d_x \\ d_y \end{bmatrix}$$

### Step 3: 노이즈 추가

가우시안 노이즈를 추가하여 실제 센서의 측정 오차를 모사한다:

$$\tilde{x}_s = x_s + \mathcal{N}(0, \sigma_p^2)$$

$$\tilde{y}_s = y_s + \mathcal{N}(0, \sigma_p^2)$$

$$\tilde{\theta}_s = \theta_s + \mathcal{N}(0, \sigma_\theta^2)$$

기본값: $\sigma_p = 0.2\text{mm}$, $\sigma_\theta = 0.05°$

## 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `odom_topic` | `/dsdbot_base_controller/odom` | wheel odometry 입력 토픽 |
| `pose_topic` | `/pgv/raw_pose` | PGV 시뮬레이션 출력 토픽 |
| `publish_rate` | 50.0 | 발행 주기 (Hz) |
| `sim_sensor_offset_x` | 0.0 | 센서 오프셋 $d_x$ (m, base_link 기준) |
| `sim_sensor_offset_y` | 0.51 | 센서 오프셋 $d_y$ (m, base_link 기준) |
| `sim_sensor_yaw_offset_deg` | 0.0 | 센서 yaw 오프셋 $\Delta\theta_{\text{yaw}}$ (deg) |
| `pos_noise_std` | 0.0002 | 위치 노이즈 표준편차 $\sigma_p$ (m) |
| `yaw_noise_std_deg` | 0.05 | yaw 노이즈 표준편차 $\sigma_\theta$ (deg) |
| `init_x` | 0.0 | odom 원점의 map x 좌표 (m) |
| `init_y` | 0.0 | odom 원점의 map y 좌표 (m) |
| `init_yaw_deg` | 0.0 | odom 원점의 map yaw (deg) |

## init 파라미터의 역할

실제 PGV는 바닥 마커로부터 절대 좌표를 읽으므로 odom과 무관하다.
dummy는 odom 기반이므로, odom 원점과 map 원점의 관계를 `init_x/y/yaw`로 설정해야 한다.

예: 센서가 map (0, 0)에서 시작하도록 하려면:

$$y_{\text{init}} = -d_y = -0.51 \quad (\theta_{\text{init}} = 0 \text{일 때})$$

이렇게 하면:

$$y_s = (-0.51) + 0 + 0.51 \cdot \cos(0) = 0$$

## map → odom TF 발행 (옵션)

`publish_tf: True`로 설정하면, 센서 데이터 발행 시 동시에 `map → odom` TF를 계산하여 발행한다.
이를 통해 dummy 환경에서는 `pgv_localization_node` 없이 단독으로 TF 체인을 완성할 수 있다.

### TF 계산 수식

노이즈 반영된 센서 데이터로부터 base_link의 map상 위치를 역산한다:

$$x_b = \tilde{x}_s - d_x \cos\tilde{\theta}_s + d_y \sin\tilde{\theta}_s$$

$$y_b = \tilde{y}_s - d_x \sin\tilde{\theta}_s - d_y \cos\tilde{\theta}_s$$

이후 odom → base_link (wheel encoder)와 합성하여 map → odom을 계산한다:

$$T_{\text{map}}^{\text{odom}} = T_{\text{map}}^{\text{base}} \cdot \left(T_{\text{odom}}^{\text{base}}\right)^{-1}$$

### /current_pose 발행

`publish_tf: True`일 때, base_link의 map상 위치를 `/current_pose` (PoseStamped) 토픽으로도 발행한다.
디버깅 및 모니터링에 활용 가능.

### 파라미터 (TF 관련)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `publish_tf` | True | map→odom TF 발행 여부 |
| `map_frame` | `map` | map 프레임 이름 |
| `odom_frame` | `odom` | odom 프레임 이름 |

## 실제 PGV와의 대응

| 항목 | 실제 PGV | Dummy PGV |
|------|---------|-----------|
| 위치 기준 | 바닥 마커 (절대) | odom + init offset |
| 센서 오프셋 | 물리적 장착 위치 | `sim_sensor_offset_x/y` |
| 노이즈 | 센서 고유 오차 | `pos_noise_std`, `yaw_noise_std_deg` |
| 출력 프레임 | map | map |
| 출력 좌표 | 센서 위치 | 센서 위치 (동일) |
