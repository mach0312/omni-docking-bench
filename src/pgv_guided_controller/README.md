# pgv_guided_controller

PGV(R4 등) 라인 센서의 `PoseStamped (x, y, yaw)` 피드백을 이용한 ROS 2 로봇 제어 패키지.
홀로노믹(메카넘/옴니)과 비홀로노믹(차동/조향) 플랫폼을 모두 지원합니다.

> 본 패키지는 [`omni-docking-bench`](../../README.md) 저장소의 일부이며,
> ICROS 2026 논문의 **기구학 기반 정밀 도킹 컨트롤러**가 구현된 핵심 모듈입니다.

---

## 포함된 노드

| 노드 (executable) | 파일 | 설명 |
|-------------------|------|------|
| `pgv_controller` | `pgv_controller_node.py` | **(x, y, yaw) 좌표 이동 컨트롤러** |

---

## pgv_controller (좌표 이동 컨트롤러)

임의의 (x, y, yaw) 목표를 받아 로봇을 해당 좌표로 이동시키는 범용 컨트롤러.

### 상태 머신

```
     goal 수신                  임계 이내
IDLE ---------> COARSE ----------------------> FINE
 ^                                               |
 |         cancel                                | 사이클 완료 후 판정
 +<------------+---------------------------------+
               |                tolerance 이내 → DONE → IDLE
               |                tolerance 밖   → FINE 재시도
```

| Phase | 설명 |
|-------|------|
| IDLE | 목표 없음, 정지 |
| COARSE | P 제어로 목표 접근 (vx, vy, w 동시 제어) |
| FINE | cubic interpolation 기반 micro 정밀 보정 |
| DONE | 도달 완료 후 IDLE 전이 |

### 인터페이스

모든 토픽/서비스 이름은 **파라미터화** 되어 있으며, launch arg `namespace` 를 통해 추가로 네임스페이스를 주입할 수 있습니다.

**Subscribe:**

| 파라미터(기본값) | 타입 | 설명 |
|------|------|------|
| `pose_topic` (`/pgv/raw_pose`) | PoseStamped | PGV 센서 포즈 |
| `goal_topic` (`/pgv_controller/goal`) | PoseStamped | (x, y, yaw) 이동 목표 |
| `forward_topic` (`/pgv_controller/go_forward`) | Bool | 수동 전진 hold |
| `backward_topic` (`/pgv_controller/go_backward`) | Bool | 수동 후진 hold |

**Publish:**

| 파라미터(기본값) | 타입 | 설명 |
|------|------|------|
| `cmd_topic` (`/tracing_vel`) | Twist | 속도 명령 |
| `state_topic` (`/pgv_controller/state`) | String | 현재 phase |

**Service:**

| 파라미터(기본값) | 타입 | 설명 |
|------|------|------|
| `cancel_service` (`/pgv_controller/cancel`) | Trigger | 목표 취소 → IDLE |

### 사용법

```bash
# 런치 (기본)
ros2 launch pgv_guided_controller pgv_controller.launch.py

# namespace 주입
ros2 launch pgv_guided_controller pgv_controller.launch.py namespace:=robot1

# 파라미터 파일 지정
ros2 launch pgv_guided_controller pgv_controller.launch.py \
  params_file:=/path/to/my_params.yaml

# 목표 발행
ros2 topic pub --once /pgv_controller/goal geometry_msgs/PoseStamped \
  '{pose: {position: {x: 1.0, y: 0.5}, orientation: {z: 0.0, w: 1.0}}}'

# 상태 모니터링
ros2 topic echo /pgv_controller/state

# 취소
ros2 service call /pgv_controller/cancel std_srvs/srv/Trigger {}
```

### 파라미터

`config/pgv_controller.params.yaml` 참고.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `pose_topic` | `/pgv/raw_pose` | 센서 포즈 토픽 |
| `cmd_topic` | `/tracing_vel` | 속도 명령 토픽 |
| `goal_topic` | `/pgv_controller/goal` | 목표 토픽 |
| `forward_topic` / `backward_topic` | — | 수동 hold 토픽 |
| `state_topic` | `/pgv_controller/state` | phase 토픽 |
| `cancel_service` | `/pgv_controller/cancel` | 취소 서비스 |
| `holonomic` | `true` | 홀로노믹 여부 |
| `kp_x` / `kp_y` / `kp_yaw` | 1.0 | P 게인 |
| `max_lin_vel` / `max_ang_vel` | 0.3 / 0.05 | 속도 상한 |
| `accel_lin` / `accel_ang` | 0.8 / 1.5 | 가속 제한 |
| `coarse_xy_threshold` | 0.01 m | COARSE→FINE 전환 거리 |
| `coarse_yaw_threshold_deg` | 5.0 | COARSE→FINE 전환 yaw 오차 |
| `fine_xy_tolerance` | 0.003 m | 최종 xy 허용오차 |
| `fine_yaw_tolerance_deg` | 2.0 | 최종 yaw 허용오차 |
| `fine_cycle_duration` | 1.0 s | micro 1사이클 시간 |
| `sensor_x_offset` / `sensor_y_offset` | 0.0 m | 센서 장착 위치 오프셋 (FINE 회전 보상에 사용) |
| `pose_timeout_sec` | 0.1 s | 센서 타임아웃 |
| `post_timeout_grace_sec` | 1.0 s | 타임아웃 회복 후 유예 시간 |
| `yaw_wrap_band_deg` | 12.0 | ±pi 경계 부호 고정 밴드 |
| `allow_reverse_heading` | `true` | ±180도 방향 허용 |
| `teleop_speed` | 0.15 m/s | 수동 hold-to-run 속도 |

### 제어 수식

#### COARSE Phase (holonomic)

3DOF 독립 P 제어. slew rate 제한 + 속도 클램프 적용.

$$e_x = x^* - x, \quad e_y = y^* - y, \quad e_\psi = \text{normalize}(\psi^* - \psi)$$

$$v_x = K_{p,x} \cdot e_x, \quad v_y = K_{p,y} \cdot e_y, \quad \omega = K_{p,\psi} \cdot e_\psi$$

$$\|\mathbf{v}\| = \sqrt{v_x^2 + v_y^2} \leq v_{\max}, \quad |\omega| \leq \omega_{\max}$$

#### COARSE Phase (non-holonomic)

목표 방향과 목표 yaw를 거리 기반으로 블렌딩:

$$\theta_{\text{goal}} = \text{atan2}(e_y, e_x), \quad e_{\theta} = \text{normalize}(\theta_{\text{goal}} - \psi)$$

$$d = \sqrt{e_x^2 + e_y^2}, \quad \alpha = \text{clamp}\left(\frac{d}{d_{\text{thresh}}}, 0, 1\right)$$

$$e_{\text{eff}} = \alpha \cdot e_{\theta} + (1 - \alpha) \cdot e_\psi$$

$$v = K_{p,x} \cdot d \cdot \cos(e_{\theta}), \quad \omega = K_{p,\psi} \cdot e_{\text{eff}}$$

멀리($\alpha \to 1$)일수록 목표 방향으로 선회, 가까이($\alpha \to 0$)일수록 최종 yaw 수렴.

#### FINE Phase (micro control)

Hermite cubic interpolation으로 부드러운 위치/각도 보정:

$$s(\tau) = 3\tau^2 - 2\tau^3, \quad \dot{s}(\tau) = 6\tau(1 - \tau), \quad \tau = \frac{t - t_s}{t_e - t_s}$$

각 축의 보정량 $\Delta x, \Delta y, \Delta\psi$는 `fine_max_delta_xy`, `fine_max_delta_yaw`로 클램프.
사이클 완료 후 tolerance 판정, 미달 시 재시도.

#### 센서 오프셋 회전 보상 (FINE)

센서가 로봇 중심에서 $(d_x, d_y)$ 만큼 떨어져 있을 때, 로봇이 $\omega$로 회전하면 센서 위치가 변한다.
이를 보상하기 위해 FINE phase에서 추가 속도를 인가한다:

$$v_{x,\text{comp}} = \omega \cdot d_y, \quad v_{y,\text{comp}} = -\omega \cdot d_x$$

유도: 센서 위치 $\mathbf{p}_s = \mathbf{p}_r + R(\psi) \cdot \mathbf{d}$ 의 시간 미분에서 로봇 회전에 의한 성분:

$$\dot{\mathbf{p}}_s\big|_{\text{rot}} = \dot{\psi} \cdot \frac{\partial R}{\partial \psi} \cdot \mathbf{d} = \omega \begin{bmatrix} -\sin\psi & -\cos\psi \\ \cos\psi & -\sin\psi \end{bmatrix} \begin{bmatrix} d_x \\ d_y \end{bmatrix}$$

로봇 프레임($\psi \approx 0$)에서 근사하면:

$$\dot{\mathbf{p}}_s\big|_{\text{rot}} \approx \begin{bmatrix} \omega \cdot d_y \\ -\omega \cdot d_x \end{bmatrix}$$

센서가 중심에 있으면($d_x = d_y = 0$) 보상 없음.

---

## 안전 메커니즘

1. **Pose timeout**: `pose_timeout_sec` 동안 센서 데이터 미수신 시 즉시 정지
2. **Grace recovery**: 센서 회복 후 `post_timeout_grace_sec` 동안 추가 정지 유지 (튀는 값 방지)
3. **Yaw wrap band**: ±pi 경계 근처 sign flip 억제
4. **Slew rate limiting**: 급격한 속도 변화 방지

---

## 디렉토리 구조

```
pgv_guided_controller/
├── pgv_guided_controller/
│   ├── pgv_controller_node.py   # (x,y,yaw) 좌표 이동 컨트롤러
│   └── __init__.py
├── launch/
│   └── pgv_controller.launch.py
├── config/
│   └── pgv_controller.params.yaml
└── test/
```

---

## 의존성

- ROS 2 Jazzy (Ubuntu 24.04)
- `rclpy`, `geometry_msgs`, `std_msgs`, `std_srvs`, `nav_msgs`

## 빌드

루트 워크스페이스에서:

```bash
cd ~/omni_docking_bench_ws
colcon build --packages-select pgv_guided_controller
source install/setup.bash
```

---

**Maintainer:** 손재락 (Jaerak Son) — sjr9017@khu.ac.kr
**License:** Apache-2.0
