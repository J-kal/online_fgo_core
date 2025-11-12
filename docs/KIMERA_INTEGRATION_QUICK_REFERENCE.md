# Kimera VIO Integration - Quick Reference

## Build Flags

### online_fgo_core
```cmake
option(ENABLE_KIMERA_INTEGRATION "Build Kimera VIO integration" OFF)
```

### Kimera-VIO
```cmake
option(ENABLE_GRAPH_TIME_CENTRIC_ADAPTER "Use GraphTimeCentric backend" OFF)
add_definitions(-DENABLE_GRAPH_TIME_CENTRIC_ADAPTER)
```

---

## Key Classes

### online_fgo_core

**GraphTimeCentricKimera** - Main graph class
- Extends `GraphTimeCentric`
- Location: `online_fgo_core/include/online_fgo_core/graph/GraphTimeCentricKimera.h`
- Key Method: `findOrCreateStateForTimestamp(timestamp)`
- Creates states at arbitrary timestamps for IMU-first operation

**KimeraIntegrationInterface** - Clean interface for adapter
- Location: `online_fgo_core/include/online_fgo_core/integration/KimeraIntegrationInterface.h`
- Facade pattern hiding graph complexity
- Used by Kimera adapter to interact with GraphTimeCentric

### Kimera-VIO

**GraphTimeCentricBackendAdapter** - Adapter in Kimera
- Location: `Kimera-VIO/include/kimera-vio/integration/GraphTimeCentricBackendAdapter.h`
- Translates VioBackend calls to GraphTimeCentric
- Handles timestamp conversion and IMU buffering

---

## Core Workflow (IMU-Only)

```
VioBackend receives IMU
        ↓
GraphTimeCentricBackendAdapter::addIMUMeasurement()
        ↓
KimeraIntegrationInterface::addIMUData()
        ↓
GraphTimeCentricKimera::addIMUMeasurements()
        ↓
[Buffer IMU measurements]

On Keyframe:
VioBackend creates keyframe
        ↓
Adapter::addKeyframeState(timestamp, pose_estimate)
        ↓
Interface::createStateAtTimestamp(timestamp)
        ↓
GraphTimeCentricKimera::findOrCreateStateForTimestamp()
        ↓
[Create X(n), V(n), B(n) keys and initial values]

On Optimization:
VioBackend triggers optimize
        ↓
Adapter::optimizeGraph()
        ↓
Interface::optimize()
        ↓
GraphTimeCentricKimera::constructFactorGraphFromTimestamps()
        ↓
[Create CombinedImuFactors between states]
        ↓
[Add GP motion priors if enabled]
        ↓
GraphTimeCentricKimera::optimizeWithExternalFactors()
        ↓
[Run ISAM2/BatchFixedLagSmoother]
        ↓
Interface returns OptimizationResult
        ↓
Adapter extracts NavState, bias, covariance
        ↓
VioBackend updates internal state
```

---

## Timestamp Conventions

- **Kimera**: `Timestamp` (nanoseconds or custom type)
- **online_fgo_core**: `double` (seconds)
- **Conversion**: In `GraphTimeCentricBackendAdapter`
  ```cpp
  double timestampToSeconds(Timestamp ts) const;
  Timestamp secondsToTimestamp(double sec) const;
  ```

---

## State Index vs State Handle

**State Index** (online_fgo_core):
- `size_t` value from `nState_` counter
- Used as index in GTSAM symbol: `X(state_idx)`, `V(state_idx)`, `B(state_idx)`

**State Handle** (interface):
```cpp
struct StateHandle {
  size_t index;      // State index
  double timestamp;  // State timestamp
  bool valid;        // Is handle valid?
};
```

---

## Parameter Mapping

### Kimera BackendParams → KimeraIntegrationParams

| Kimera | online_fgo_core |
|--------|----------------|
| `smartNoiseSigma_` | → smart factor noise |
| `imu_params.gyro_noise_` | → `gyro_noise_sigma` |
| `imu_params.acc_noise_` | → `accel_noise_sigma` |
| `imu_params.n_gravity_` | → `gravity` |
| `horizon_` | → `smoother_lag` |
| `linearizationMode_` | → ISAM2 params |

---

## Data Structure Conversions

### IMU Measurement

**Kimera ImuAccGyr**:
```cpp
struct ImuAccGyr {
  Timestamp timestamp;
  gtsam::Vector3 acc;
  gtsam::Vector3 gyro;
};
```

**online_fgo_core IMUMeasurement**:
```cpp
struct IMUMeasurement {
  TimeStamp timestamp;    // seconds
  Vector3 accLin;         // linear accel
  Vector3 gyro;           // angular velocity
  Vector3 accRot;         // rotational accel (optional)
  double dt;              // time delta
  Matrix3 accLinCov;      // covariance
  Matrix3 gyroCov;
};
```

### State

**Kimera VioNavState**:
```cpp
VioNavState {
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity;
  gtsam::imuBias::ConstantBias bias;
};
```

**online_fgo_core State**:
```cpp
struct State {
  gtsam::NavState state;  // pose + velocity
  gtsam::imuBias::ConstantBias imuBias;
  Matrix poseVar;
  Matrix velVar;
  Matrix imuBiasVar;
  // ... additional fields
};
```

---

## Key Methods by Functionality

### State Creation
```cpp
// GraphTimeCentricKimera
size_t findOrCreateStateForTimestamp(double ts, bool create = true);
bool hasStateAtTimestamp(double ts, double tol = 0.001);
size_t getStateIndexAtTimestamp(double ts, double tol = 0.001);

// Interface
StateHandle createStateAtTimestamp(double ts);
StateHandle getStateAtTimestamp(double ts);
```

### IMU Handling
```cpp
// GraphTimeCentricKimera
bool addIMUMeasurements(const vector<IMUMeasurement>& imu);
bool addIMUFactorBetweenStates(size_t i, size_t j, const vector<IMUMeasurement>& imu);

// Interface
bool addIMUData(double ts, const Vector3& acc, const Vector3& gyro, double dt);
size_t addIMUDataBatch(const vector<double>& ts, ...);

// Adapter
bool addIMUMeasurement(const ImuAccGyr& imu);
size_t addIMUMeasurements(const vector<ImuAccGyr>& imu);
```

### Optimization
```cpp
// GraphTimeCentricKimera
StatusGraphConstruction constructFactorGraphFromTimestamps(const vector<double>& ts);
double optimizeWithExternalFactors(State& new_state);

// Interface
OptimizationResult optimize();
bool optimizeAndGetLatestState(NavState& state, ConstantBias& bias);

// Adapter
bool optimizeGraph();
```

### Result Retrieval
```cpp
// GraphTimeCentricKimera
optional<Pose3> getOptimizedPose(size_t idx);
optional<Vector3> getOptimizedVelocity(size_t idx);
optional<ConstantBias> getOptimizedBias(size_t idx);
optional<Matrix> getStateCovariance(size_t idx);

// Interface
optional<NavState> getOptimizedState(StateHandle handle);
optional<ConstantBias> getOptimizedBias(StateHandle handle);
optional<Matrix> getStateCovariance(StateHandle handle);

// Adapter
optional<NavState> getStateAtTime(Timestamp ts);
optional<NavState> getLatestState();
optional<ConstantBias> getLatestIMUBias();
```

---

## Error Handling

### Adapter Not Compiled
```cpp
#ifndef ENABLE_GRAPH_TIME_CENTRIC_ADAPTER
  if (backend_params.use_graph_time_centric) {
    LOG(FATAL) << "GraphTimeCentric adapter requested but not compiled!";
    // Or fallback to native backend
  }
#endif
```

### State Not Found
```cpp
StateHandle handle = interface->getStateAtTimestamp(ts);
if (!handle) {
  LOG(ERROR) << "No state at timestamp " << ts;
  return false;
}
```

### Optimization Failure
```cpp
OptimizationResult result = interface->optimize();
if (!result.success) {
  LOG(ERROR) << "Optimization failed: " << result.error_message;
  // Use last valid state
  return last_valid_state_;
}
```

---

## Threading Model

- **VioBackend**: Runs on backend thread
- **Adapter**: Called from VioBackend thread
- **GraphTimeCentric**: Single-threaded graph operations
- **Thread Safety**: Adapter uses mutex for buffer access

```cpp
// In Adapter
std::lock_guard<std::mutex> lock(buffer_mutex_);
imu_buffer_.push_back(imu);
```

---

## Testing Strategy

### Unit Tests

**online_fgo_core**:
```cpp
// test_graphtimecentric_kimera.cpp
TEST(GraphTimeCentricKimera, StateCreation) { ... }
TEST(GraphTimeCentricKimera, IMUFactorCreation) { ... }
TEST(GraphTimeCentricKimera, GPPriors) { ... }
TEST(GraphTimeCentricKimera, Optimization) { ... }
```

**Interface**:
```cpp
// test_kimera_integration_interface.cpp
TEST(KimeraIntegrationInterface, Initialization) { ... }
TEST(KimeraIntegrationInterface, StateManagement) { ... }
TEST(KimeraIntegrationInterface, IMUData) { ... }
```

**Adapter**:
```cpp
// test_adapter.cpp (in Kimera-VIO)
TEST(Adapter, Initialization) { ... }
TEST(Adapter, TimestampConversion) { ... }
TEST(Adapter, IMUBuffering) { ... }
TEST(Adapter, Optimization) { ... }
```

### Integration Test

```cpp
// Use EuRoC MH_01_easy dataset
// Run first 100 frames, IMU-only
// Compare pose estimates with baseline
```

---

## Configuration Example

### online_fgo_core YAML
```yaml
graph_time_centric_kimera:
  timestamp_match_tolerance: 0.001
  create_states_at_imu_rate: true
  imu_state_frequency: 200
  
  use_combined_imu_factor: true
  
  add_gp_motion_priors: true
  gp_type: "WNOJ"
  gp_qc: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
  
  smoother_lag: 5.0
  smoother_type: "ISAM2FixedLag"
```

### Kimera-VIO YAML
```yaml
backend:
  use_graph_time_centric: true
  
  # Path to online_fgo_core config
  graph_time_centric:
    config_path: "path/to/online_fgo_config.yaml"
```

---

## Debugging Tips

### Enable Verbose Logging
```cpp
// In GraphTimeCentricKimera
graphBaseParamPtr_->verbose = true;
kimeraParams_.addGPMotionPriors = true;  // To see GP prior info
```

### Check State Creation
```cpp
auto timestamps = graph->getAllStateTimestamps();
LOG(INFO) << "Created " << timestamps.size() << " states";
for (auto ts : timestamps) {
  LOG(INFO) << "State at " << std::fixed << ts;
}
```

### Check Factor Graph
```cpp
graph->print("Graph: ");
LOG(INFO) << "Factor count: " << graph->size();
```

### Monitor Optimization Time
```cpp
double opt_time = adapter->getLastOptimizationTime();
LOG(INFO) << "Optimization took " << opt_time << " seconds";
```

---

## ISAM2 Optimization Details

### How ISAM2 Fits In

The incremental fixed-lag smoother uses ISAM2 as its core optimizer:

```
GraphTimeCentricKimera::optimize()
        ↓
[Build factor graph from buffered states]
        ↓
IncrementalFixedLagSmoother::update()
        ↓
gtsam::ISAM2::update(factors, values)
        ↓
[Incremental Bayes Tree update]
        ↓
calculateEstimate() → optimized states
```

### ISAM2 Parameters (GraphBase.cpp)

Configured via your parameter file:

```yaml
Optimizer:
  smootherType: "IncrementalFixedLag"
  ISAM2:
    optimizationParams: "GN"         # or "DOGLEG"
    relinearizeThreshold: 0.1        # When to relinearize
    relinearizeSkip: 10              # Check every N updates
    factorization: "CHOLESKY"        # or "QR"
    enableDetailedResults: false     # Debug info
    findUnusedFactorSlots: false     # Memory optimization
```

### What Gets Optimized

For each Kimera keyframe at timestamp `t`:

**Variables** (GTSAM Keys):
- `X(i)`: Pose at state index i
- `V(i)`: Velocity at state index i  
- `B(i)`: IMU bias at state index i

Where `i = findOrCreateStateForTimestamp(t)`

**Factors** (Constraints):
1. **CombinedImuFactor**: Between consecutive states
   - Preintegrates all IMU measurements between timestamps
   - Provides odometry constraint
   
2. **GPMotorPrior**: Motion smoothness (if enabled)
   - Gaussian Process prior on acceleration
   - Enforces physically plausible motion
   
3. **PriorFactor**: On initial state
   - Sets global reference frame
   - From Kimera's initial estimate

### Optimization Behavior

**Incremental Updates**:
- Only new variables and affected neighbors are updated
- Old states within `smootherLag` window stay in optimization
- States older than `smootherLag` are marginalized out

**Relinearization**:
- ISAM2 tracks how far states drift from linearization point
- When drift > `relinearizeThreshold`, affected states are relinearized
- Checked every `relinearizeSkip` updates for efficiency

**Fixed-Lag Marginalization**:
```
Time: ───[t-5s]────[t-3s]────[t-1s]────[t]───>
      ↑           ↑          ↑         ↑
      Marginalized (if smootherLag=3s)
      (absorbed into linear prior)
                   └─────── Active optimization window ─────┘
```

### What ISAM2 Returns

After `optimize()`, you get:

```cpp
// Optimized state values
Pose3 optimizedPose = values.at<Pose3>(X(i));
Vector3 optimizedVel = values.at<Vector3>(V(i));
imuBias::ConstantBias optimizedBias = values.at<imuBias::ConstantBias>(B(i));

// Marginal covariances (if computed)
Matrix poseCovariance = marginals.marginalCovariance(X(i));
```

### Performance Characteristics

**Typical Update Times** (depends on window size and relinearization):
- No relinearization: 5-20 ms
- With relinearization: 50-200 ms
- First update (batch): 100-500 ms

**Memory Usage**:
```
States in window = smootherLag / dt
Memory ≈ states × (pose + velocity + bias + factors)
Example: 3s lag @ 10Hz = 30 states ≈ 5-10 MB
```

**Scaling**:
- Complexity: O(m·k²) where m=new states, k=affected states
- With fixed-lag: k stays constant → constant-time updates!

### Debugging ISAM2

Enable detailed results to monitor optimization:

```cpp
// In config
ISAM2:
  enableDetailedResults: true

// Check result
ISAM2Result result = smoother_->getISAM2Result();
LOG(INFO) << "Variables relinearized: " << result.variablesRelinearized;
LOG(INFO) << "Cliques updated: " << result.cliques;
LOG(INFO) << "Error: " << result.errorAfter.value();
```

**See**: `ISAM2_TUTORIAL.md` for comprehensive ISAM2 explanation.

---

## Common Pitfalls

1. **Timestamp Mismatch**: Ensure consistent timestamp units (seconds)
2. **Missing dt**: IMU measurements need dt field for preintegration
3. **State Not Found**: Check tolerance when querying by timestamp
4. **Covariance Not Available**: Ensure `getMarginals()` called after optimization
5. **Thread Safety**: Don't call graph methods from multiple threads
6. **ISAM2 Divergence**: Check `relinearizeThreshold` if optimization diverges
7. **Slow Optimization**: Reduce `smootherLag` or increase `relinearizeSkip`

---

## Performance Tuning

### Reduce Optimization Time
- Decrease `smoother_lag` (fewer states in window)
- Use `ISAM2FixedLag` instead of `BatchFixedLag`
- Disable GP priors if not needed

### Reduce Memory Usage
- Limit `smoother_lag`
- Clear old buffers regularly

### Improve Accuracy
- Increase GP prior QC covariance tuning
- Use WNOJFull or SingerFull for better motion model
- Tune IMU noise parameters

---

## References

- Main implementation plan: `KIMERA_INTEGRATION_PLAN.md`
- Skeleton summary: `KIMERA_INTEGRATION_SKELETON_SUMMARY.md`
- Original tasks: `doc slop/todo.md`
- GraphTimeCentric reference: `GraphTimeCentric.cpp`

---

**Quick Start**: See `KIMERA_INTEGRATION_PLAN.md` Phase 2 for step-by-step implementation guide
