# Online FGO Core - ROS-Agnostic Factor Graph Optimization Library

## Overview

`online_fgo_core` is a framework-agnostic C++ library for real-time factor graph optimization (FGO), extracted from the original ROS2-dependent `online_fgo` package. It provides core functionality for GNSS/INS/LIO fusion using GTSAM, with support for time-centric and sensor-centric graph construction strategies.

## Features

- **Framework Agnostic**: No dependencies on ROS, can be used in standalone C++ applications
- **GraphTimeCentric**: Time-synchronized factor graph construction
- **GraphSensorCentric**: Sensor-driven factor graph construction  
- **Modular Design**: Clean separation between graph, solver, integrator, and factor components
- **Extensible**: Easy to add new sensors and factors through plugin-style integrators
- **High Performance**: Optimized with GTSAM for real-time operation

## Dependencies

### Required
- **GTSAM** >= 4.0: Georgia Tech Smoothing and Mapping library
- **GTSAM_UNSTABLE**: Unstable features from GTSAM
- **Eigen3** >= 3.3.5: Linear algebra library
- **Boost**: System libraries
- **GeographicLib**: Geographic coordinate conversions
- **TBB**: Intel Threading Building Blocks for parallelization

### Optional
- **yaml-cpp**: For YAML parameter file loading (recommended)
- **OpenMP**: For additional parallelization
- **PCL**: Point Cloud Library (required for LIO integrators)
- **OpenCV**: Computer vision (required for visual integrators)

## Building

### Standard CMake Build

```bash
cd online_fgo_core
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### Build Options

- `BUILD_TESTS=ON`: Build unit tests
- `BUILD_EXAMPLES=ON`: Build example applications
- `WITH_YAML=ON`: Enable YAML parameter loading (default: ON)

Example:
```bash
cmake -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
```

## Usage

### Standalone Application

```cpp
#include <online_fgo_core/interface/ApplicationInterface.h>
#include <online_fgo_core/graph/GraphTimeCentric.h>

int main() {
    // Create standalone application
    auto app = std::make_shared<fgo::core::StandaloneApplication>("my_fgo_app");
    
    // Configure parameters
    auto& params = app->getParameterServer();
    params->setDouble("GNSSFGO.Graph.smootherLag", 5.0);
    params->setBool("GNSSFGO.Graph.publishResiduals", false);
    
    // Create graph
    fgo::graph::GraphTimeCentric graph(*app);
    
    // Initialize and use...
    // ... add measurements, construct graph, optimize ...
    
    return 0;
}
```

### With Parameter File

```cpp
auto app = std::make_shared<fgo::core::StandaloneApplication>("my_app");
app->getParameterServer()->loadFromYAML("config/params.yaml");
```

## Architecture

### Core Interfaces

The library provides abstract interfaces to decouple core logic from framework-specific implementations:

- **LoggerInterface**: Platform-agnostic logging (info, warn, error, debug)
- **TimeInterface**: Framework-independent time representation
- **ParameterInterface**: Configuration parameter access
- **PublisherInterface**: Data output abstraction
- **ApplicationInterface**: Unified access to all interfaces

### Main Components

1. **Graph** (`graph/`): Factor graph construction and management
   - `GraphBase`: Common graph functionality
   - `GraphTimeCentric`: Time-synchronized graph construction
   - `GraphSensorCentric`: Sensor-driven graph construction

2. **Solver** (`solver/`): GTSAM solver wrappers
   - `FixedLagSmoother`: Base smoother interface
   - `IncrementalFixedLagSmoother`: ISAM2-based incremental solver
   - `BatchFixedLagSmoother`: Batch optimization solver

3. **Integrator** (`integrator/`): Sensor-specific measurement processing
   - `IntegratorBase`: Base class for all integrators
   - `GNSSTCIntegrator`: GNSS tight-coupling
   - `IMUPreIntegrator`: IMU preintegration
   - `LIOIntegrator`: LiDAR-inertial odometry
   - And more...

4. **Factor** (`factor/`): GTSAM factors for various measurements
   - Motion factors (GP priors, constant acceleration, etc.)
   - GNSS factors (pseudorange, carrier phase, etc.)
   - Visual factors
   - Inertial factors

5. **Data** (`data/`): Data structures and buffers
   - State representation
   - Measurement types
   - Circular buffers

6. **Utils** (`utils/`): Utility functions
   - Navigation tools
   - GP utilities
   - Pose utilities

## Adapters

The core library can be wrapped with framework-specific adapters:

- **online_fgo_ros2**: ROS2 adapter (rclcpp-based nodes)
- **online_fgo_ros1**: ROS1 adapter (roscpp-based nodes)

See respective packages for integration examples.

## Examples

(To be added with BUILD_EXAMPLES=ON)

## Testing

```bash
cd build
cmake -DBUILD_TESTS=ON ..
make -j$(nproc)
ctest
```

## License

Apache License 2.0 - See LICENSE file for details

## Authors

- Haoming Zhang (h.zhang@irt.rwth-aachen.de)
- Institute of Automatic Control, RWTH Aachen University

## Citation

If you use this library in your research, please cite:

```bibtex
@article{zhang2024onlinefgo,
  title={Online Factor Graph Optimization for GNSS/INS/LIO Integration},
  author={Zhang, Haoming and others},
  journal={TBD},
  year={2024}
}
```

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## Support

For questions and issues, please open an issue on the GitHub repository.
