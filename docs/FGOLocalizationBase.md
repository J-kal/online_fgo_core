    # FGOLocalizationBase - ROS-Agnostic Port

## Overview

`FGOLocalizationBase` is a framework-agnostic base class for FGO (Factor Graph Optimization) based localization systems. It was ported from the original `GNSSFGOLocalizationBase` class in the gnssFGO package, removing all ROS dependencies and replacing them with generic interfaces.

## Key Changes from Original

### 1. **Removed ROS Dependencies**
- Replaced `rclcpp::Node` inheritance with `ApplicationInterface*` composition
- Removed direct ROS message type dependencies
- Replaced ROS-specific logging with `LoggerInterface`
- Replaced ROS parameter server with `ParameterInterface`
- Replaced ROS time with `TimeInterface` (via `ApplicationInterface`)

### 2. **Interface-Based Design**
The class now depends on abstract interfaces defined in `online_fgo_core/interface/`:
- `ApplicationInterface` - Main application abstraction
- `LoggerInterface` - Framework-agnostic logging
- `ParameterInterface` - Parameter loading/retrieval
- `PublisherInterface` - Message publishing abstraction
- `TimeInterface` - Time management (via ApplicationInterface)

### 3. **Virtual Publishing Methods**
Publishing methods are now virtual and provide default no-op implementations:
- `publishFGOState()` - Publishes FGO state
- `publishPositionAsNavFix()` - Publishes position as lat/lon/alt
- `publishPose()` - Publishes pose with covariance
- `publishVelocity()` - Publishes velocity with covariance
- `publishTiming()` - Publishes timing information
- `publishError()` - Publishes error to ground truth

Framework-specific derived classes (in `online_fgo_ros1` or `online_fgo_ros2`) should override these methods to publish in the appropriate message format.

## Architecture

```
┌─────────────────────────────────────┐
│   FGOLocalizationBase (core)       │
│   - Framework agnostic              │
│   - Uses ApplicationInterface       │
│   - Virtual publish methods         │
└──────────────┬──────────────────────┘
               │
               │ inherits
               │
┌──────────────▼──────────────────────┐
│  GNSSFGOLocalizationROS1 (ros1)    │
│  - Inherits from both:              │
│    * FGOLocalizationBase            │
│    * ros::NodeHandle (or wrapper)   │
│  - Implements ROS1-specific pubs    │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  GNSSFGOLocalizationROS2 (ros2)    │
│  - Inherits from both:              │
│    * FGOLocalizationBase            │
│    * rclcpp::Node                   │
│  - Implements ROS2-specific pubs    │
└─────────────────────────────────────┘
```

## Usage

### In online_fgo_core (framework-agnostic)

```cpp
#include "online_fgo_core/FGOLocalizationBase.h"

// Application interface must be provided
fgo::core::ApplicationInterface* app = /* ... */;

// Create localization base
auto fgo_base = std::make_shared<fgo::FGOLocalizationBase>(app);

// Initialize common parameters
fgo_base->initializeCommon();
```

### In online_fgo_ros1 (ROS1-specific)

```cpp
#include "online_fgo_core/FGOLocalizationBase.h"
#include "online_fgo_ros1/ROS1ApplicationInterface.h"

class GNSSFGOLocalizationROS1 : public fgo::FGOLocalizationBase {
public:
    GNSSFGOLocalizationROS1(ros::NodeHandle& nh)
        : FGOLocalizationBase(new ROS1ApplicationInterface(nh)) {
        
        // Setup ROS1-specific publishers
        pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
        
        initializeCommon();
    }
    
protected:
    // Override publishing methods for ROS1
    void publishFGOState(const fgo::data::State& state, const std::string& topic) override {
        // Convert to ROS1 message and publish
        auto msg = convertToROS1FGOStateMsg(state);
        state_pub_.publish(msg);
    }
    
    void publishPose(const fgo::data::State& state) override {
        geometry_msgs::PoseWithCovarianceStamped msg;
        // Fill message...
        pose_pub_.publish(msg);
    }
    
private:
    ros::Publisher pose_pub_;
    ros::Publisher state_pub_;
};
```

### In online_fgo_ros2 (ROS2-specific)

```cpp
#include "online_fgo_core/FGOLocalizationBase.h"
#include "online_fgo_ros2/ROS2ApplicationInterface.h"

class GNSSFGOLocalizationROS2 : public fgo::FGOLocalizationBase, public rclcpp::Node {
public:
    GNSSFGOLocalizationROS2(const rclcpp::NodeOptions& options)
        : rclcpp::Node("gnss_fgo", options),
          FGOLocalizationBase(new ROS2ApplicationInterface(this)) {
        
        // Setup ROS2-specific publishers
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
        
        initializeCommon();
    }
    
protected:
    // Override publishing methods for ROS2
    void publishFGOState(const fgo::data::State& state, const std::string& topic) override {
        auto msg = convertToROS2FGOStateMsg(state);
        state_pub_->publish(msg);
    }
    
    void publishPose(const fgo::data::State& state) override {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        // Fill message...
        pose_pub_->publish(msg);
    }
    
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr state_pub_;
};
```

## Files Created/Modified

### Created:
- `online_fgo_core/include/online_fgo_core/FGOLocalizationBase.h` - Header file with class definition
- `online_fgo_core/src/FGOLocalizationBase.cpp` - Implementation file

### Modified:
- `online_fgo_core/CMakeLists.txt` - Added `online_fgo_localization` library target

## Dependencies

The `FGOLocalizationBase` depends on:
- `online_fgo_interface` - Interface abstractions
- `online_fgo_data` - Data types
- `online_fgo_graph` - Graph implementations (GraphTimeCentric)
- `online_fgo_utils` - Utility functions
- `online_fgo_integrator` - Integrators
- GTSAM - Factor graph optimization
- Eigen3 - Linear algebra
- GeographicLib - Geographic transformations
- Boost - Various utilities

## Integration with GraphTimeCentric

`GraphTimeCentric` already uses `ApplicationInterface&` instead of ROS node, so it seamlessly works with `FGOLocalizationBase`. The graph is instantiated in `initializeCommon()`:

```cpp
graph_ = std::make_shared<fgo::graph::GraphTimeCentric>(appPtr_);
```

This allows the Kimera integration mentioned in the TODO to reference `FGOLocalizationBase` instead of the non-existent `GNSSFGOLocalizationBase`.

## Next Steps for Kimera Integration

As noted in `todo.md`, the `GraphTimeCentricKimera` constructor can now be:

```cpp
GraphTimeCentricKimera(fgo::FGOLocalizationBase& node)
  : GraphTimeCentric(node.getApplicationPtr()) {
  // Kimera-specific initialization
}
```

This provides access to:
- Parameters via `node.getParamPtr()`
- Graph via `node.getGraphPtr()`
- Sensor calibration via `node.getSensorCalibManagerPtr()`
- State initialization status via `node.isStateInitialized()`

## Building

The library is automatically built when building `online_fgo_core`:

```bash
cd online_fgo_core
mkdir build && cd build
cmake ..
make
```

To install:
```bash
make install
```

The installed library can be used in downstream packages via:
```cmake
find_package(online_fgo_core REQUIRED)
target_link_libraries(your_target online_fgo_core::online_fgo_localization)
```
