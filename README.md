# Simple_calibration_using_trajectories

A hand-eye calibration tool that solves the classic **AX=XB** calibration problem using Ceres optimizer. This project estimates the rigid transformation (rotation + translation) between two sensors based on their trajectory data.

## Overview

The AX=XB calibration problem is fundamental in robotics and computer vision for determining the spatial relationship between two sensors. Given corresponding pose pairs from sensor A and sensor B moving together, this tool computes the transformation matrix `T_A_B` that relates frame A to frame B.

### Application Scenarios

- **LiDAR-RTK Calibration**: Estimate the extrinsic parameters between LiDAR and RTK/GPS sensors
- **Robot Eye-in-Hand**: Camera-to-robot-end-effector calibration
- **Multi-Sensor Fusion**: Spatial alignment of any two rigidly-mounted sensors

## Algorithm Principle

### Mathematical Formulation

The AX=XB equation describes the hand-eye calibration constraint:

```
A * X = X * B
```

Where:
- **A**: Relative motion of sensor A between two poses (`T_A⁻¹ * T_A'`)
- **B**: Relative motion of sensor B between corresponding poses (`T_B⁻¹ * T_B'`)
- **X**: Unknown transformation to be solved (`T_A_B`)

### Geometric Interpretation

For any pair of consecutive poses:
```
T_A_i⁻¹ * T_A_j = R_A, t_A  (relative motion of sensor A)
T_B_i⁻¹ * T_B_j = R_B, t_B  (relative motion of sensor B)
```

The constraint `A * X = X * B` means that transforming the relative motion by X yields the same result regardless of which frame it's expressed in.

### Optimization

The problem is formulated as a non-linear least squares optimization using Ceres Solver:

**Rotation Residual:**
```
q_err = qA * qX * qB⁻¹ * qX⁻¹  →  minimize ||angle-axis(q_err)||
```

**Translation Residual:**
```
t_res = R_A * tX + tA - R_X * tB - tX  →  minimize ||t_res||
```

## Directory Structure

```
calib_AXXB/
├── CMakeLists.txt              # Build configuration
├── build.sh                    # Build script
├── clear.sh                    # Clean script
├── README.md                   # This file
├── config/
│   └── calib.yaml              # Configuration file
├── include/
│   └── hand_eye_calibrator.h   # Class definition
├── src/
│   └── hand_eye_calibrator.cpp # Core implementation
├── app/
│   └── calib_lidar_rtk.cpp     # Main entry point
└── data/
    ├── lidar_pose.txt          # Sample sensor A trajectory
    ├── rtk_pose.txt            # Sample sensor B trajectory
    └── calib_result.txt        # Output result
```

## Dependencies

- **Eigen3** - Linear algebra library
- **Ceres Solver** - Non-linear optimization
- **PCL (Point Cloud Library)** - Point cloud processing
- **yaml-cpp** - YAML configuration parsing
- **glog** - Google logging library

## Build

```bash
cd calib_AXXB
chmod +x build.sh clear.sh
./build.sh
```

The executable will be generated at `bin/calib_AXXB_lidar_rtk`.

## Usage

### Configuration

Edit `config/calib.yaml`:

```yaml
hand_eye_calibrator:
  # Optimization options
  only_optimize_rotation: false    # true=rotation only, false=6DoF
  max_iterations: 1000             # Maximum iterations
  residual_threshold: 0.00000001   # Convergence threshold

  # Data alignment
  time_tolerance_seconds: 0.1      # Max time diff for pose matching
  skip: 1                          # Frame skip for motion computation

  # Initial guess (4x4 transformation matrix, row-major)
  # A better initial guess helps convergence
  T_A_B_init: [1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0]

  # Input/Output
  A_poses_file: "data/lidar_pose.txt"
  B_poses_file: "data/rtk_pose.txt"
  save_result_file: "data/calib_result.txt"
```

### Input Data Format

Trajectory files should contain poses with the following format:

```
timestamp(s)  x  y  z  qx  qy  qz  qw
```

Example (`data/lidar_pose.txt`):
```
1600000000.0  0.0  0.0  0.0  0.0  0.0  0.0  1.0
1600000000.1  0.1  0.0  0.0  0.0  0.0  0.0  1.0
1600000000.2  0.2  0.0  0.0  0.0  0.0  0.0  1.0
...
```

Where:
- `timestamp`: Unix timestamp in seconds
- `x, y, z`: Position (meters)
- `qx, qy, qz, qw`: Orientation as quaternion

### Run

```bash
./bin/calib_AXXB_lidar_rtk config/calib.yaml
```

### Output

The result file contains two 4x4 transformation matrices:

```
T_A_B (estimated transform from frame A to frame B):
    R  |  t
   ----+----
    0  |  0

T_B_A (inverse transform):
    R  |  t
   ----+----
    0  |  0
```

## Demo

The repository includes sample data for testing:

**Ground Truth** (for demo data):
```
T_rtk_lidar:
0.736, -0.677, 0.0, 0.0,
0.677,  0.736, 0.0, 0.0,
0.0,    0.0,   1.0, 0.130,
0.0,    0.0,   0.0, 1.000;
```

## Algorithm Details

### Processing Pipeline

1. **Load Poses**: Read trajectory data from two sensor files
2. **Time Alignment**: Interpolate poses to align timestamps using SLERP (rotation) and linear interpolation (translation)
3. **Build Relative Motions**: Compute relative transformations between pose pairs with configurable skip
4. **Ceres Optimization**: Minimize the AX=XB residual using automatic differentiation
5. **Output Result**: Save the estimated transformation matrices

### Key Implementation Features

- **Quaternion Normalization**: Ensures valid rotation representations
- **Safe SLERP Interpolation**: Handles edge cases for smooth pose interpolation
- **Configurable Skip**: Allows using non-consecutive frame pairs for motion computation
- **Convergence Callback**: Monitors optimization progress and termination

## Parameters Guide

| Parameter | Recommended | Description |
|-----------|-------------|-------------|
| `only_optimize_rotation` | `false` | Set `true` only when translation is known |
| `max_iterations` | 500-2000 | Increase if convergence not reached |
| `residual_threshold` | 1e-8 to 1e-11 | Smaller = stricter convergence |
| `time_tolerance_seconds` | 0.05-0.2 | Depends on trajectory timestamp precision |
| `skip` | 1-5 | Larger = more motion between frames, fewer pairs |
| `T_A_B_init` | Close to true value | Critical for large initial errors |

## License

BSD License - See LICENSE file for details.

## Acknowledgments

Based on the classic hand-eye calibration formulation and implemented with [Ceres Solver](http://ceres-solver.org/).
