# Release Process (binary plugin)

This document describes how the maintainer (re)builds and re-publishes the
pre-built `libdouble_steering_drive_controller.so` that ships with this
repository via GitHub Releases.

External users do **not** need to follow this procedure — they only need
to download the `.so` asset into `libs/double_steering_drive_controller/lib/`
before running `colcon build`.

## 1. Pre-conditions

The binary is compiled in a **pristine Ubuntu 24.04 + ROS 2 Jazzy** environment
so that the ABI matches what end users will have after a `ros-jazzy-desktop`
install.

| Property | Value |
|----------|-------|
| Base image | `ros:jazzy-ros-base` or Ubuntu 24.04 + apt `ros-jazzy-ros-base` |
| Architecture | `x86_64` (linux/amd64) |
| Compiler | g++-14 (Ubuntu 24.04 default) |
| Standard library | libstdc++ 14.x |
| CMake | ≥ 3.22 |
| Flags | `-O2 -DNDEBUG -fPIC` (default colcon release) |

An `arm64` variant is not officially shipped. If you need one, repeat the
procedure inside an `arm64` base image and publish the asset with an
`_arm64.so` suffix.

## 2. Build the `.so`

The controller source lives **outside** this public repository. Clone it
alongside (the maintainer has access):

```bash
mkdir -p ~/dsd_build_ws/src
cd ~/dsd_build_ws/src
git clone git@<private-host>:<org>/double_steering_drive_controller.git
cd ~/dsd_build_ws

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -yir
colcon build --packages-select double_steering_drive_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The artefacts live under:

```
install/double_steering_drive_controller/
├── lib/libdouble_steering_drive_controller.so           # ship this
├── share/double_steering_drive_controller/
│   └── double_steering_drive_plugin.xml                 # already committed, compare only
└── include/double_steering_drive_controller/*.hpp       # optional: ship as tarball
```

## 3. Verify ABI / plugin metadata

```bash
SO=install/double_steering_drive_controller/lib/libdouble_steering_drive_controller.so

# Sanity check: the class symbol exists
nm -D --defined-only "$SO" | grep DoubleSteeringDriveController

# Sanity check: linked libs look like Jazzy
ldd "$SO" | grep -E "ros|rcl|controller_interface"
```

The plugin XML that ships with the open-source stub under
`libs/double_steering_drive_controller/double_steering_drive_plugin.xml` must
declare the **same** `<class name>` / `<class type>` as the closed-source
plugin XML. A mismatch will make `controller_manager` fail to load the controller.
Run a diff whenever the controller internals change:

```bash
diff install/double_steering_drive_controller/share/double_steering_drive_controller/double_steering_drive_plugin.xml \
     <this-repo>/libs/double_steering_drive_controller/double_steering_drive_plugin.xml
```

## 4. Stage the assets

```bash
RELEASE_DIR=/tmp/release
mkdir -p "$RELEASE_DIR"
cp "$SO" "$RELEASE_DIR/libdouble_steering_drive_controller.so"

# Optional header tarball (only if downstream users want to compile
# against the plugin; omit otherwise).
tar -czf "$RELEASE_DIR/double_steering_drive_controller_include.tar.gz" \
  -C install/double_steering_drive_controller include

cd "$RELEASE_DIR"
sha256sum libdouble_steering_drive_controller.so >> SHA256SUMS
sha256sum double_steering_drive_controller_include.tar.gz >> SHA256SUMS 2>/dev/null || true
cat SHA256SUMS
```

## 5. Publish the release

```bash
TAG=v0.1.0
cd <this-repo>
git tag -a "$TAG" -m "ICROS 2026 reproducibility release $TAG"
git push origin "$TAG"

gh release create "$TAG" \
  "$RELEASE_DIR/libdouble_steering_drive_controller.so" \
  "$RELEASE_DIR/double_steering_drive_controller_include.tar.gz" \
  "$RELEASE_DIR/SHA256SUMS" \
  --title "Omni-Docking-Bench $TAG" \
  --notes-file RELEASE_NOTES.md
```

The release notes must include:

1. Target platform (Ubuntu 24.04 / ROS 2 Jazzy / x86_64)
2. Toolchain (g++-14, libstdc++ 14.x)
3. `sha256sum` of every attached asset
4. The commit SHA of the private controller source at build time
5. The commit SHA of this repository

## 6. Update the fetch instructions

After publishing, bump the URL in:

- `README.md` (§ *Quick start / Native*)
- `libs/double_steering_drive_controller/README.md`

and commit those as part of the release PR.
