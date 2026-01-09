# Decode Estimator

A factor graph-based 2D robot pose estimator using GTSAM and iSAM2. Fuses odometry with bearing measurements to known AprilTag landmarks for accurate localization that counters odometry drift.

## Features

- **iSAM2 incremental optimization** - efficient 100Hz updates
- **Custom bearing factor** - bearing-only measurements to known landmarks
- **Odometry fusion** - BetweenFactor for relative pose changes
- **Optional Rerun visualization** - real-time trajectory and measurement display
- **Thread-safe** - mutex-protected for multi-threaded use

## Dependencies

### Required
- CMake 3.16+
- C++17 compiler
- Boost (serialization, system, filesystem, thread, program_options, date_time, timer, chrono, regex)
  - GTSAM and Eigen are built locally as static libs (no system installs required).
  - GTSAM is built without TBB.

### Optional
- Rerun SDK (for visualization)
- Google Test (for unit tests)

## Installation

### Ubuntu/Debian

```bash
# Build essentials + Boost
sudo apt-get install build-essential cmake libboost-all-dev

# Google Test (optional, for tests)
sudo apt-get install libgtest-dev
```

### Building from Source

```bash
# Clone the repository
cd /path/to/decode-estimator

# Build (Eigen + GTSAM are built locally as ExternalProject deps)
./scripts/build.sh

# Build with tests
./scripts/build.sh --tests

# Run tests (if built)
./scripts/test.sh

# Run example
./build/examples/simple_localization

# Build and run example with Rerun visualization
./scripts/run_example.sh
```

### Cross-compiling for Compute Module 4

```bash
# Cross-compile with toolchain
CM4_SYSROOT=/path/to/sysroot \
CM4_TOOLCHAIN_PREFIX=aarch64-linux-gnu \
./scripts/build_cm4.sh
```

### Building for Android

The project includes a JNI interface for use on Android. All dependencies (Eigen, GTSAM, Boost) are automatically cross-compiled for Android.

**Prerequisites:**
- Android NDK (r21 or later recommended)
- Set `ANDROID_NDK_HOME` or `ANDROID_NDK` environment variable

**Build:**

```bash
# Basic Android build (arm64-v8a, API 21)
./scripts/build.sh --android

# Specify ABI and API level
./scripts/build.sh --android --android-abi arm64-v8a --android-api 28

# Specify custom NDK path
./scripts/build.sh --android --android-ndk /path/to/ndk

# Available ABIs: armeabi-v7a, arm64-v8a, x86, x86_64
./scripts/build.sh --android --android-abi armeabi-v7a
```

**Output:**
- `build/libdecode_estimator_jni.so` - JNI shared library for Android

**Note:** The first Android build will take 15-30 minutes as it compiles Boost, Eigen, and GTSAM from source. Subsequent builds are much faster due to CMake caching.

**JNI Interface:**

The JNI interface is available through `com.decode.estimator.PoseEstimatorBridge`:

```java
// Create with default config
PoseEstimatorBridge bridge = new PoseEstimatorBridge();
long handle = bridge.nativeCreate();

// OR create with custom noise parameters
long handle = bridge.nativeCreateWithConfig(
    0.05,  // prior_sigma_xy (m)
    0.02,  // prior_sigma_theta (rad)
    0.02,  // odom_sigma_xy (m)
    0.01,  // odom_sigma_theta (rad)
    0.05,  // default_bearing_sigma (rad, ~3 degrees)
    0.2    // default_distance_sigma (m)
);

// Initialize with landmarks
int[] tagIds = {1, 2, 3};
double[] landmarkX = {0.0, 5.0, 5.0};
double[] landmarkY = {5.0, 5.0, 0.0};
bridge.nativeInitialize(handle, tagIds, landmarkX, landmarkY, 0.0, 0.0, 0.0);

// Process odometry
bridge.nativeProcessOdometry(handle, dx, dy, dtheta, timestamp);

// Add bearing measurement
bridge.nativeAddBearingMeasurement(handle, tagId, bearing, uncertainty, timestamp);

// Add distance measurement (optional)
bridge.nativeAddDistanceMeasurement(handle, tagId, distance, uncertainty, timestamp);

// Update and get pose
bridge.nativeUpdate(handle);
double[] pose = bridge.nativeGetCurrentEstimate(handle);  // [x, y, theta, timestamp]

// Get pose with covariance (more expensive)
double[] poseWithCov = bridge.nativeGetCurrentEstimateWithCovariance(handle);
// Returns [x, y, theta, timestamp, cov[0], cov[1], ..., cov[8]]
// Covariance is 3x3 matrix in row-major order

// Clean up
bridge.nativeDestroy(handle);
```

**Noise Parameter Guidelines:**
- `prior_sigma_xy`: Initial position uncertainty (typical: 0.05m = 5cm)
- `prior_sigma_theta`: Initial heading uncertainty (typical: 0.02rad ≈ 1.1°)
- `odom_sigma_xy`: Odometry position noise per update (typical: 0.02m = 2cm)
- `odom_sigma_theta`: Odometry heading noise per update (typical: 0.01rad ≈ 0.6°)
- `default_bearing_sigma`: Bearing measurement uncertainty (typical: 0.05rad ≈ 3°)
- `default_distance_sigma`: Distance measurement uncertainty (typical: 0.2m = 20cm)

Tune these based on your sensor characteristics and testing.

### Notes

- The build scripts fetch Eigen/GTSAM from their upstream git repositories; network access is required the first time.
- Tests are host-only and disabled by default.

## Usage

### Basic Usage

```cpp
#include <decode_estimator/pose_estimator.hpp>

// Configure estimator
decode::EstimatorConfig config;
config.odom_sigma_xy = 0.02;        // 2cm per step
config.odom_sigma_theta = 0.01;     // ~0.5 degrees per step
config.default_bearing_sigma = 0.05; // ~3 degrees

// Create estimator
decode::PoseEstimator estimator(config);

// Define AprilTag landmarks (from field map)
std::vector<decode::Landmark> landmarks = {
    {1, 0.0, 5.0},    // Tag 1 at (0, 5)
    {2, 5.0, 5.0},    // Tag 2 at (5, 5)
    {3, 5.0, 0.0},    // Tag 3 at (5, 0)
};

// Initialize with known starting pose
estimator.initialize(landmarks, 0.0, 0.0, 0.0);

// Main loop (100 Hz)
while (running) {
    // Process odometry (dx, dy, dtheta in robot frame)
    decode::OdometryMeasurement odom;
    odom.dx = get_odom_dx();
    odom.dy = get_odom_dy();
    odom.dtheta = get_odom_dtheta();
    odom.timestamp = get_timestamp();

    estimator.processOdometry(odom);

    // Add bearing measurements when AprilTags are detected
    for (const auto& detection : get_apriltag_detections()) {
        decode::BearingMeasurement bearing;
        bearing.tag_id = detection.id;
        bearing.bearing_rad = detection.bearing;  // In robot frame
        bearing.uncertainty_rad = detection.uncertainty;
        bearing.timestamp = get_timestamp();

        estimator.addBearingMeasurement(bearing);
    }

    // Perform iSAM2 update and get corrected pose
    decode::PoseEstimate pose = estimator.update();

    // Use pose.x, pose.y, pose.theta for control
}
```

### API Reference

#### EstimatorConfig

| Parameter | Default | Description |
|-----------|---------|-------------|
| `relinearize_threshold` | 0.01 | iSAM2 relinearization threshold |
| `relinearize_skip` | 1 | Relinearize every N updates |
| `prior_sigma_xy` | 0.05 | Initial position uncertainty (m) |
| `prior_sigma_theta` | 0.02 | Initial heading uncertainty (rad) |
| `odom_sigma_xy` | 0.02 | Odometry position noise per step (m) |
| `odom_sigma_theta` | 0.01 | Odometry heading noise per step (rad) |
| `default_bearing_sigma` | 0.05 | Default bearing uncertainty (rad) |
| `enable_visualization` | false | Enable Rerun visualization |

#### PoseEstimator Methods

| Method | Description |
|--------|-------------|
| `initialize(landmarks, x, y, theta)` | Initialize with landmarks and starting pose |
| `reset()` | Reset to uninitialized state |
| `processOdometry(odom)` | Add odometry measurement |
| `addBearingMeasurement(bearing)` | Add bearing measurement |
| `update()` | Perform iSAM2 update, returns pose |
| `getCurrentEstimate()` | Get current pose without update |
| `getCurrentEstimateWithCovariance()` | Get pose with uncertainty |
| `getTrajectory()` | Get all poses in graph |

## Factor Graph Architecture

```
[Prior] → X(0) ─[Odom]→ X(1) ─[Odom]→ X(2) ─[Odom]→ X(3) ...
            │              │              │
       [Bearing]      [Bearing]      [Bearing]
            ↓              ↓              ↓
        Tag(id=1)      Tag(id=3)      Tag(id=1)
       (known pos)    (known pos)    (known pos)
```

- **Prior Factor**: Anchors initial pose X(0)
- **BetweenFactor**: Connects consecutive poses with odometry
- **BearingToKnownLandmarkFactor**: Custom unary factor for bearing to known landmark

## Performance Notes

- iSAM2 provides O(log n) updates vs O(n³) for batch optimization
- Typical update time: <1ms for 100+ poses
- Use `Release` build for ~10x speedup over `Debug`
- Avoid `getCurrentEstimateWithCovariance()` in tight loops (marginal covariance is expensive)

## License

MIT License
