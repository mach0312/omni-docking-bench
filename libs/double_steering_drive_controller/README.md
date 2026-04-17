# double_steering_drive_controller (binary plugin)

본 디렉토리는 **ros2_control** 용 `double_steering_drive_controller` 플러그인의
**이진 배포 전용** 패키지다. 소스 코드는 공개 레포에 포함되지 않으며,
사전 컴파일된 `.so` 파일 + pluginlib XML + 공개 헤더만을 제공한다.

이는 논문 재현성 목적으로 충분하다. ROS 2 controller_manager 는
`pluginlib` 를 통해 `.so` 를 `dlopen()` 하여 런타임에 로드하므로,
사용자는 해당 클래스 이름(`double_steering_drive_controller/DoubleSteeringDriveController`)
을 `controllers.yaml` 의 `type:` 필드에 지정하는 것만으로 동일한 제어기를
사용할 수 있다.

## 대상 플랫폼

- **OS**: Ubuntu 24.04 (Noble)
- **ROS 2**: Jazzy (Desktop)
- **아키텍처**: `x86_64` (amd64)
- **컴파일러**: `g++-14` (Ubuntu 24.04 기본)
- **ABI**: libstdc++ 14.x

다른 플랫폼(예: arm64, 22.04)에서는 동작을 보장하지 않는다.
해당 환경에서는 원저자에게 별도 요청하여 재컴파일된 바이너리를 받아야 한다.

## 파일 배치

`colcon build` 이전에 아래와 같이 바이너리를 배치해야 한다.

```
libs/double_steering_drive_controller/
├── CMakeLists.txt
├── package.xml
├── double_steering_drive_plugin.xml
├── README.md                   # 본 문서
├── include/                    # 공개 헤더 (선택)
│   └── double_steering_drive_controller/
│       └── *.hpp
└── lib/
    └── libdouble_steering_drive_controller.so   # ← GitHub Release 에서 다운로드
```

`lib/libdouble_steering_drive_controller.so` 가 없으면 빌드가 실패하며
CMake 가 친절한 에러 메시지를 출력한다.

## 다운로드

GitHub Release 페이지에서 최신 자산을 받는다.

```bash
cd libs/double_steering_drive_controller
curl -L -o lib/libdouble_steering_drive_controller.so \
  https://github.com/mach0312/omni-docking-bench/releases/download/vX.Y.Z/libdouble_steering_drive_controller.so

# (필요 시) 공개 헤더
curl -L -o include.tar.gz \
  https://github.com/mach0312/omni-docking-bench/releases/download/vX.Y.Z/double_steering_drive_controller_include.tar.gz
tar -xzf include.tar.gz && rm include.tar.gz
```

## SHA-256 검증

바이너리의 무결성은 릴리스 노트에 기록된 SHA-256 해시로 검증한다.

```bash
sha256sum lib/libdouble_steering_drive_controller.so
# expected: <64 자리 hex> (릴리스 노트 참조)
```

## 라이선스

해당 바이너리는 Apache-2.0 라이선스로 배포된다.
이진 재배포는 본 레포의 [LICENSE](../../LICENSE) 조건을 따른다.

## 재컴파일 / 직접 빌드

해당 컨트롤러의 소스 코드는 본 공개 레포의 범위에 포함되지 않으나,
ros2_control 플러그인 규격은 공개되어 있으며,
동일 인터페이스(`controller_interface::ControllerInterface`)를 구현하는
자체 컨트롤러로 대체하는 것도 가능하다.
상세 재빌드 절차는 루트 `docs/RELEASE_PROCESS.md` 를 참고.
