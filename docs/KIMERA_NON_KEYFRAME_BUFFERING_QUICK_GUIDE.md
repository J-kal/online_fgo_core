# Quick Usage Guide: Non-Keyframe Buffering in Kimera VIO

## Quick Start

### 1. Update Your VioBackend Integration

Replace the current keyframe-only logic with the buffering system:

**Before:**
```cpp
// Old code - discards non-keyframes
void VioBackend::processFrame(const Frame& frame) {
  if (shouldBeKeyframe(frame)) {
    // Only keyframes are added
    adapter_->addKeyframeState(frame.timestamp, pose, velocity, bias);
    adapter_->optimizeGraph();
  }
  // Non-keyframes are discarded!
}
```

**After:**
```cpp
// New code - buffers non-keyframes
void VioBackend::processFrame(const Frame& frame) {
  if (shouldBeKeyframe(frame)) {
    // Keyframe: processes buffer + adds keyframe
    adapter_->addKeyframeState(frame.timestamp, pose, velocity, bias);
    adapter_->optimizeGraph();
  } else {
    // Non-keyframe: buffer for later
    adapter_->bufferNonKeyframeState(frame.timestamp, pose, velocity, bias);
  }
}
```

### 2. API Reference

#### Buffer a Non-Keyframe
```cpp
bool GraphTimeCentricBackendAdapter::bufferNonKeyframeState(
    const Timestamp& timestamp,        // Frame timestamp
    const gtsam::Pose3& pose,          // Pose estimate
    const gtsam::Vector3& velocity,    // Velocity estimate
    const gtsam::imuBias::ConstantBias& bias  // IMU bias
);
```
- **Returns**: `true` if successfully buffered
- **Thread-safe**: Yes (internal mutex)
- **When to call**: When VioBackend determines frame is NOT a keyframe

#### Add Keyframe (Updated)
```cpp
void GraphTimeCentricBackendAdapter::addKeyframeState(
    const Timestamp& timestamp,
    const gtsam::Pose3& pose,
    const gtsam::Vector3& velocity,
    const gtsam::imuBias::ConstantBias& bias
);
```
- **Behavior**: 
  1. Processes all buffered non-keyframes (chronologically)
  2. Adds the keyframe state
  3. Clears the buffer
- **Thread-safe**: Yes
- **When to call**: When VioBackend determines frame IS a keyframe

## Complete Example

### Minimal Integration in VioBackend

```cpp
#ifdef ENABLE_GRAPH_TIME_CENTRIC_ADAPTER
#include "kimera-vio/integration/GraphTimeCentricBackendAdapter.h"
#endif

class VioBackend {
private:
#ifdef ENABLE_GRAPH_TIME_CENTRIC_ADAPTER
  std::unique_ptr<GraphTimeCentricBackendAdapter> adapter_;
#endif

public:
  VioBackend(/* params */) {
#ifdef ENABLE_GRAPH_TIME_CENTRIC_ADAPTER
    if (backend_params_.use_graph_time_centric) {
      adapter_ = std::make_unique<GraphTimeCentricBackendAdapter>(
          backend_params_, imu_params_);
      
      if (!adapter_->initialize()) {
        LOG(FATAL) << "Failed to initialize GraphTimeCentric adapter";
      }
    }
#endif
  }
  
  BackendOutput::UniquePtr spinOnce(const BackendInput& input) {
    // ... existing VIO processing ...
    
#ifdef ENABLE_GRAPH_TIME_CENTRIC_ADAPTER
    if (backend_params_.use_graph_time_centric) {
      processFrameWithAdapter(input);
    } else {
      processFrameWithSmoother(input);  // Legacy path
    }
#else
    processFrameWithSmoother(input);
#endif
    
    // ... return output ...
  }
  
private:
  void processFrameWithAdapter(const BackendInput& input) {
    // Extract state estimates
    gtsam::Pose3 pose = /* ... from VIO ... */;
    gtsam::Vector3 velocity = /* ... from VIO ... */;
    gtsam::imuBias::ConstantBias bias = /* ... from VIO ... */;
    
    // Add IMU measurements
    for (const auto& imu_meas : input.imu_accgyr_) {
      adapter_->addIMUMeasurement(imu_meas);
    }
    
    // Keyframe decision
    bool is_keyframe = shouldBeKeyframe(input);
    
    if (is_keyframe) {
      LOG(INFO) << "Frame " << input.frame_id_ << " is KEYFRAME";
      
      // This will process buffered non-keyframes + add keyframe
      adapter_->addKeyframeState(
          input.timestamp_, pose, velocity, bias);
      
      // Trigger optimization
      if (!adapter_->optimizeGraph()) {
        LOG(ERROR) << "Optimization failed!";
      }
      
      // Get optimized state
      auto optimized_pose = adapter_->getOptimizedPoseAtTime(
          timestampToSeconds(input.timestamp_));
      
      if (optimized_pose.has_value()) {
        LOG(INFO) << "Optimized pose: " << optimized_pose.value();
      }
      
    } else {
      LOG(INFO) << "Frame " << input.frame_id_ << " is NON-KEYFRAME (buffering)";
      
      // Buffer for later addition
      adapter_->bufferNonKeyframeState(
          input.timestamp_, pose, velocity, bias);
    }
  }
  
  bool shouldBeKeyframe(const BackendInput& input) {
    // Your existing keyframe selection logic
    // e.g., based on disparity, rotation, translation, etc.
    return /* ... */;
  }
};
```

## Configuration

### CMake Build Flags
```cmake
# Enable GraphTimeCentric adapter
set(ENABLE_GRAPH_TIME_CENTRIC_ADAPTER ON)

# In Kimera-VIO CMakeLists.txt
if(ENABLE_GRAPH_TIME_CENTRIC_ADAPTER)
  add_definitions(-DENABLE_GRAPH_TIME_CENTRIC_ADAPTER)
  
  # Link online_fgo_core
  target_link_libraries(kimera_vio
    online_fgo_core
  )
endif()
```

### Runtime Parameters
```yaml
# In backend params (e.g., BackendParams.yaml)
backend:
  use_graph_time_centric: true  # Enable adapter at runtime
```

## Monitoring and Debugging

### Check Buffer Size
```cpp
size_t buffer_size = adapter_->getNumBufferedStates();
LOG(INFO) << "Current buffer size: " << buffer_size;
```

### Statistics Logging
```cpp
std::string stats = adapter_->getStatistics();
LOG(INFO) << "Adapter statistics:\n" << stats;
```

### Expected Log Output

**When buffering non-keyframes:**
```
[INFO] GraphTimeCentricBackendAdapter: buffered non-keyframe state at t=1630000000.100000 (buffer size: 1)
[INFO] GraphTimeCentricBackendAdapter: buffered non-keyframe state at t=1630000000.200000 (buffer size: 2)
[INFO] GraphTimeCentricBackendAdapter: buffered non-keyframe state at t=1630000000.300000 (buffer size: 3)
```

**When keyframe arrives:**
```
[INFO] GraphTimeCentricBackendAdapter: processing keyframe at t=1630000000.400000
[INFO] GraphTimeCentricBackendAdapter: processing 3 buffered non-keyframe states
[INFO] GraphTimeCentricBackendAdapter: added buffered state 1 at timestamp 1630000000.100000
[INFO] GraphTimeCentricBackendAdapter: added buffered state 2 at timestamp 1630000000.200000
[INFO] GraphTimeCentricBackendAdapter: added buffered state 3 at timestamp 1630000000.300000
[INFO] GraphTimeCentricBackendAdapter: created keyframe state 4 at timestamp 1630000000.400000 (total states: 4)
```

## Testing

### Unit Test Example
```cpp
TEST(GraphTimeCentricBackendAdapter, BufferAndProcessNonKeyframes) {
  // Setup
  GraphTimeCentricBackendAdapter adapter(backend_params, imu_params);
  ASSERT_TRUE(adapter.initialize());
  
  // Buffer 3 non-keyframes
  Timestamp t1 = 1000000000;  // 1.0 sec
  Timestamp t2 = 1100000000;  // 1.1 sec
  Timestamp t3 = 1200000000;  // 1.2 sec
  Timestamp t4 = 1300000000;  // 1.3 sec (keyframe)
  
  gtsam::Pose3 pose1 = /* ... */;
  gtsam::Vector3 vel1 = /* ... */;
  gtsam::imuBias::ConstantBias bias1 = /* ... */;
  
  // Buffer non-keyframes
  EXPECT_TRUE(adapter.bufferNonKeyframeState(t1, pose1, vel1, bias1));
  EXPECT_TRUE(adapter.bufferNonKeyframeState(t2, pose2, vel2, bias2));
  EXPECT_TRUE(adapter.bufferNonKeyframeState(t3, pose3, vel3, bias3));
  
  // Add keyframe (should process buffer)
  adapter.addKeyframeState(t4, pose4, vel4, bias4);
  
  // Verify all states added
  EXPECT_EQ(adapter.getNumStates(), 4);
  
  // Verify buffer cleared
  EXPECT_EQ(adapter.getNumBufferedStates(), 0);
}
```

## Troubleshooting

### Problem: Buffer never clears
**Cause**: No keyframes being detected  
**Solution**: Check your `shouldBeKeyframe()` logic. Ensure keyframes are detected periodically (e.g., every 0.5-1 seconds).

### Problem: States added out of order
**Cause**: Timestamps not monotonically increasing  
**Solution**: This is handled automatically! Buffered states are sorted by timestamp before addition.

### Problem: High memory usage
**Cause**: Too many non-keyframes between keyframes  
**Solution**: Adjust keyframe selection to trigger more frequently. Typical: 5-10 non-keyframes per keyframe.

### Problem: Optimization slow
**Cause**: Too many states in graph  
**Solution**: 
1. Use fixed-lag smoother to limit history
2. Adjust `smoother_lag` parameter
3. Consider marginalization

## Performance Expectations

| Metric | Typical Value |
|--------|--------------|
| Non-keyframes per keyframe | 5-10 |
| Buffer overhead | < 1 ms |
| Memory per buffered state | ~176 bytes |
| Total memory (1000 keyframes) | ~1-2 MB |
| Optimization time increase | < 5% |

## Next Steps

1. ✅ Integrate into your VioBackend
2. ✅ Test with synthetic data
3. ✅ Validate on EuRoC dataset
4. ✅ Monitor buffer usage
5. ✅ Tune keyframe selection

## Additional Resources

- **Full documentation**: `KIMERA_NON_KEYFRAME_BUFFERING.md`
- **Integration plan**: `KIMERA_INTEGRATION_PLAN.md`
- **Implementation summary**: `KIMERA_INTEGRATION_IMPLEMENTATION_SUMMARY.md`
- **API reference**: Header files in `Kimera-VIO/include/kimera-vio/integration/`

---

**Questions?** Check the detailed documentation or review the implementation in:
- `GraphTimeCentricBackendAdapter.h` - API declarations
- `GraphTimeCentricBackendAdapter.cpp` - Implementation details

