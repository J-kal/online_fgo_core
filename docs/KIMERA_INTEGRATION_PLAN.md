  # Kimera VIO Integration with GraphTimeCentric - Implementation Plan

## Overview
This document outlines the integration of Kimera VIO backend with online_fgo_core's GraphTimeCentric framework, creating an IMU-first factor graph that uses timestamp-indexed states. The integration focuses initially on IMU factors and provides a clear path to add visual features (smart factors) later.

## Architecture

### Design Principles
1. **IMU-First**: States are created at IMU measurement timestamps or regular time intervals
2. **Timestamp-Indexed**: All states are indexed by timestamp in keyTimestampMap_
3. **Modular Integration**: Clean separation between Kimera and online_fgo_core via adapter pattern
4. **Compile-Time Optional**: Build flags control whether integration is compiled
5. **Runtime Toggle**: Backend can switch between native Kimera and GraphTimeCentric modes

### Component Structure
```
online_fgo_core/
├── include/online_fgo_core/
│   ├── graph/
│   │   └── GraphTimeCentricKimera.h          (NEW: Kimera-specific graph)
│   └── integration/
│       └── KimeraIntegrationInterface.h      (NEW: Interface for Kimera)
└── src/
    ├── graph/
    │   └── GraphTimeCentricKimera.cpp        (NEW)
    └── integration/
        └── KimeraIntegrationInterface.cpp    (NEW)

Kimera-VIO/
├── include/kimera-vio/
│   └── integration/
│       └── GraphTimeCentricBackendAdapter.h  (NEW: Adapter to online_fgo_core)
└── src/
    └── integration/
        └── GraphTimeCentricBackendAdapter.cpp (NEW)
```

## Phase 1: Build System & Scaffolding

### 1.1 online_fgo_core CMake Changes

File: `online_fgo_core/CMakeLists.txt`
- Add option: `option(ENABLE_KIMERA_INTEGRATION "Build Kimera VIO integration" OFF)`
- Add integration subdirectory when enabled
- Export integration targets

File: `online_fgo_core/src/graph/CMakeLists.txt`
- Conditionally add `GraphTimeCentricKimera.cpp` when `ENABLE_KIMERA_INTEGRATION=ON`

### 1.2 Kimera-VIO CMake Changes

File: `Kimera-VIO/CMakeLists.txt`
- Add option: `option(ENABLE_GRAPH_TIME_CENTRIC_ADAPTER "Use GraphTimeCentric backend" OFF)`
- Add preprocessor define: `ENABLE_GRAPH_TIME_CENTRIC_ADAPTER`
- Add integration subdirectory when enabled
- Find online_fgo_core package when enabled

## Phase 2: Core Interfaces (IMU-Only)

### 2.1 GraphTimeCentricKimera Class

**Purpose**: Extends GraphTimeCentric with Kimera-specific functionality

**Key Responsibilities**:
- State creation at arbitrary timestamps (not just IMU arrivals)
- IMU preintegration between timestamp-indexed states
- GP motion priors for IMU-driven states
- Interface for external (Kimera) factor insertion
- Landmark/smart-factor management (Phase 3)

**Key Methods**:
```cpp
// State management
size_t findOrCreateStateForTimestamp(double timestamp, bool create_if_missing = true);
bool hasStateAtTimestamp(double timestamp, double tolerance = 0.001);
size_t getStateIndexAtTimestamp(double timestamp, double tolerance = 0.001);

// IMU integration
bool addIMUMeasurements(const std::vector<IMUMeasurement>& imu_measurements);
bool constructFactorGraphFromTimestamps(const std::vector<double>& timestamps);

// GP motion priors
void addGPMotionPriorsForStates(const std::vector<size_t>& state_indices);

// External factor interface
bool addExternalFactor(const gtsam::NonlinearFactor::shared_ptr& factor, 
                       const std::vector<size_t>& state_indices);

// Optimization
double optimizeWithExternalFactors(fgo::data::State& new_state);
```

### 2.2 KimeraIntegrationInterface

**Purpose**: Interface class that Kimera adapter uses to interact with online_fgo_core

**Key Methods**:
```cpp
// Initialization
bool initialize(const KimeraIntegrationParams& params);

// State creation
StateHandle createStateAtTimestamp(double timestamp);
StateHandle getStateAtTimestamp(double timestamp);

// IMU data
bool addIMUData(double timestamp, const Eigen::Vector3d& accel, 
                const Eigen::Vector3d& gyro, double dt);

// Factor insertion (for future visual factors)
bool addFactor(const FactorData& factor_data);

// Optimization trigger
OptimizationResult optimize();

// Result retrieval
gtsam::NavState getOptimizedState(StateHandle handle);
gtsam::Matrix getStateCovariance(StateHandle handle);
```

### 2.3 GraphTimeCentricBackendAdapter (Kimera-side)

**Purpose**: Adapter in Kimera-VIO that translates VioBackend calls to GraphTimeCentric

**Key Responsibilities**:
- Translate Kimera's BackendInput to GraphTimeCentric format
- Buffer IMU measurements and forward to GraphTimeCentric
- Create states at Kimera keyframe timestamps
- Retrieve optimization results and update VioBackend state
- (Future) Translate smart factors to GraphTimeCentric format

**Key Methods**:
```cpp
// Initialization
bool initialize(const BackendParams& backend_params, 
                const ImuParams& imu_params);

// State management  
bool addKeyframeState(Timestamp timestamp, const gtsam::Pose3& pose_estimate);

// IMU handling
bool addIMUMeasurement(const ImuMeasurement& imu_measurement);
bool preintegrateIMUBetweenStates(Timestamp t_i, Timestamp t_j);

// Optimization
bool optimizeGraph();

// Result retrieval
gtsam::NavState getStateAtTime(Timestamp t);
gtsam::imuBias::ConstantBias getIMUBias();
gtsam::Matrix getStateCovariance();

// (Future Phase 3)
bool addSmartFactor(const SmartFactorData& smart_factor_data);
```

## Phase 3: Implementation Details

### 3.1 GraphTimeCentricKimera Implementation

**State Creation Logic**:
```cpp
size_t GraphTimeCentricKimera::findOrCreateStateForTimestamp(double timestamp, bool create) {
  // 1. Search keyTimestampMap_ for existing state within tolerance
  //    - Use currentKeyIndexTimestampMap_ for efficient lookup
  
  // 2. If found, return existing state index
  
  // 3. If not found and create_if_missing:
  //    a. Increment nState_
  //    b. Create keys: X(nState_), V(nState_), B(nState_)
  //    c. Query predicted state from currentPredictedBuffer_
  //    d. Insert initial values into values_
  //    e. Update keyTimestampMap_ and currentKeyIndexTimestampMap_
  //    f. Return nState_
  
  // 4. Handle edge cases: thread safety, duplicate insertion
}
```

**IMU Factor Creation**:
- Reuse GraphTimeCentric's existing CombinedImuFactor creation
- Between consecutive timestamp-indexed states
- Use preIntegratorParams_ from base class
- Integrate measurements that fall between state timestamps

**GP Motion Prior Addition**:
- Add GP prior factors between consecutive states
- Support WNOA, WNOJ, WNOJFull, Singer, SingerFull
- Anchor on IMU-driven state timestamps
- Use QcGPMotionPriorFull from parameters

### 3.2 Integration Flow

**Initialization Sequence**:
1. VioBackend checks `use_graph_time_centric` parameter
2. If enabled and compiled, create GraphTimeCentricBackendAdapter
3. Adapter creates KimeraIntegrationInterface
4. Interface creates GraphTimeCentricKimera instance
5. Initialize with parameters from VioBackend

**Runtime Flow (IMU-Only)**:
1. VioBackend receives BackendInput with IMU measurements
2. If using adapter:
   - Forward IMU measurements to adapter
   - Adapter buffers and sends to GraphTimeCentricKimera
3. At keyframe timestamp:
   - VioBackend requests state creation at keyframe time
   - Adapter calls findOrCreateStateForTimestamp()
4. When optimization triggered:
   - Adapter calls GraphTimeCentricKimera::optimize()
   - Results retrieved and translated back to VioBackend format

### 3.3 Data Translation

**Timestamp Conversion**:
- Kimera uses `Timestamp` (nanoseconds or specific type)
- GraphTimeCentric uses `double` (seconds)
- Adapter performs conversion: `timestamp_seconds = timestamp_ns / 1e9`

**IMU Measurement Translation**:
```cpp
// Kimera: ImuAccGyr with timestamp
// online_fgo_core: IMUMeasurement struct

fgo::data::IMUMeasurement translateIMU(const ImuAccGyr& kimera_imu) {
  fgo::data::IMUMeasurement fgo_imu;
  fgo_imu.timestamp = TimeStamp(kimera_imu.timestamp / 1e9);
  fgo_imu.accLin = kimera_imu.acc;
  fgo_imu.gyro = kimera_imu.gyro;
  fgo_imu.dt = compute_dt_from_buffer();
  // ... set covariances
  return fgo_imu;
}
```

**State Translation**:
```cpp
// GraphTimeCentric -> VioBackend
gtsam::Pose3 pose = result.at<gtsam::Pose3>(X(state_idx));
gtsam::Vector3 velocity = result.at<gtsam::Vector3>(V(state_idx));
gtsam::imuBias::ConstantBias bias = result.at<gtsam::imuBias::ConstantBias>(B(state_idx));

VioNavState kimera_state(pose, velocity, bias);
```

## Phase 4: Testing Strategy

### 4.1 Unit Tests

**GraphTimeCentricKimera Tests** (`online_fgo_core/tests/`):
- `test_state_creation.cpp`: State creation at arbitrary timestamps
- `test_imu_factor_creation.cpp`: IMU factor insertion between states
- `test_gp_prior_addition.cpp`: GP motion prior factors
- `test_timestamp_indexing.cpp`: Timestamp lookup and indexing

**Adapter Tests** (`Kimera-VIO/test/`):
- `test_adapter_initialization.cpp`: Adapter setup and initialization
- `test_imu_translation.cpp`: IMU data format translation
- `test_state_translation.cpp`: State result translation
- `test_optimization_trigger.cpp`: Optimization flow

### 4.2 Integration Tests

**IMU-Only Pipeline Test**:
- Use EuRoC dataset (MH_01_easy)
- Run first 100 frames with only IMU
- Compare pose estimates with baseline
- Verify optimization convergence

**Timing Test**:
- Measure state creation overhead
- Measure optimization time vs native Kimera
- Verify real-time capability

### 4.3 Regression Tests

- Ensure native Kimera backend still works when adapter disabled
- Ensure online_fgo_core existing functionality unchanged

## Phase 5: Future Extensions (Not in Initial Implementation)

### 5.1 Smart Factor Integration (Phase 3 from todo.md)

**Strategy**: Port Kimera's smart factor management into GraphTimeCentricKimera

**Components**:
- `LandmarkId -> SmartFactor` mapping
- Slot management for smart factors
- Feature track to landmark conversion
- Cheirality checking and retriangulation

**Implementation**:
- `addLandmarksToGraph()`: Create smart factors from feature tracks
- `updateSmartFactorSlots()`: Manage slot lifecycle
- `cleanCheiralityLmk()`: Handle invalid observations

### 5.2 Loop Closure

- Interface for adding loop closure factors
- Between non-consecutive states
- Use timestamp indexing for state lookup

### 5.3 Multi-Sensor Fusion

- Leverage existing integrator framework in GraphBase
- Add visual measurements as another integrator
- GPS/barometer factors through integrators

## Implementation Checklist

### Phase 1: Build System ✓
- [ ] Add ENABLE_KIMERA_INTEGRATION option to online_fgo_core
- [ ] Add integration subdirectory structure to online_fgo_core
- [ ] Add ENABLE_GRAPH_TIME_CENTRIC_ADAPTER option to Kimera-VIO
- [ ] Add integration subdirectory structure to Kimera-VIO
- [ ] Update CMake to conditionally compile integration code

### Phase 2: IMU-Only Skeleton ✓
- [ ] Create GraphTimeCentricKimera.h header
- [ ] Create GraphTimeCentricKimera.cpp implementation
- [ ] Create KimeraIntegrationInterface.h
- [ ] Create KimeraIntegrationInterface.cpp
- [ ] Create GraphTimeCentricBackendAdapter.h
- [ ] Create GraphTimeCentricBackendAdapter.cpp
- [ ] Implement state creation at arbitrary timestamps
- [ ] Implement IMU measurement buffering and forwarding
- [ ] Implement IMU factor creation between timestamp-indexed states

### Phase 3: VioBackend Integration
- [ ] Add use_graph_time_centric parameter to BackendParams
- [ ] Add adapter instantiation in VioBackend constructor
- [ ] Route IMU measurements through adapter when enabled
- [ ] Route optimization calls through adapter
- [ ] Retrieve and translate optimization results

### Phase 4: Testing
- [ ] Write GraphTimeCentricKimera unit tests
- [ ] Write adapter unit tests
- [ ] Create IMU-only integration test
- [ ] Verify timing and performance
- [ ] Regression test native Kimera backend

### Phase 5: Documentation
- [ ] Update online_fgo_core README
- [ ] Update Kimera-VIO README
- [ ] Add usage examples
- [ ] Document parameter configuration

## Key Design Decisions

### 1. Why Adapter Pattern?
- Clean separation of concerns
- Kimera and online_fgo_core remain independent
- Easy to maintain and test
- Can disable at compile time with zero overhead

### 2. Why Timestamp-Indexed States?
- Flexible state creation at arbitrary times
- Supports both IMU-driven and keyframe-driven states
- Easy GP interpolation between states
- Natural for multi-sensor fusion

### 3. Why IMU-First?
- IMU provides continuous state evolution
- Visual measurements are sparse
- GP priors work naturally with IMU timestamps
- Easier to add visual factors later to existing IMU states

### 4. State Creation Strategy
- Create states at IMU timestamps OR keyframe timestamps
- Use findOrCreateStateForTimestamp() to avoid duplicates
- Tolerance-based timestamp matching (default 1ms)
- Predicted state from currentPredictedBuffer_ as initial value

## Parameter Configuration

### online_fgo_core Parameters (GraphTimeCentricKimera)
```yaml
graph_time_centric_kimera:
  # State creation
  timestamp_match_tolerance: 0.001  # seconds
  create_states_at_imu_rate: true
  imu_state_frequency: 200  # Hz
  
  # IMU factors
  use_combined_imu_factor: true
  
  # GP motion priors
  add_gp_motion_priors: true
  gp_type: "WNOJ"  # WNOA, WNOJ, WNOJFull, Singer, SingerFull
  gp_qc: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
  
  # Optimization
  smoother_lag: 5.0  # seconds
  smoother_type: "ISAM2FixedLag"
```

### Kimera-VIO Parameters
```yaml
backend:
  use_graph_time_centric: true  # Enable GraphTimeCentric backend
  
  # When using GraphTimeCentric:
  graph_time_centric:
    online_fgo_core_config_path: "path/to/online_fgo_config.yaml"
```

## Error Handling

### Adapter Errors
- If ENABLE_GRAPH_TIME_CENTRIC_ADAPTER not defined but use_graph_time_centric=true:
  - Log fatal error: "GraphTimeCentric adapter requested but not compiled"
  - Exit or fallback to native backend

### State Creation Errors
- If timestamp outside valid range:
  - Log error and return invalid state handle
- If values_ already contains key:
  - Catch ValuesKeyAlreadyExists, return existing state index

### Optimization Errors
- Wrap solver_->update() in try-catch
- On optimization failure, log error and return last valid state
- Maintain state consistency

## Performance Considerations

### Memory
- State creation adds keys to values_ and keyTimestampMap_
- Monitor memory growth with high-frequency states
- Use smoother_lag to limit state history

### Computation
- State lookup in keyTimestampMap_ is O(log n)
- Consider caching recent state lookups
- GP factors more expensive than simple priors

### Real-Time
- Ensure optimization completes within budget
- Monitor optimization time
- Adjust smoother_lag if optimization too slow

## References

- GraphTimeCentric.cpp: Base implementation for reference
- GraphBase.h: Base class interface and utilities
- VioBackend.cpp: Kimera's native backend implementation
- online_fgo_core integrator framework: Multi-sensor pattern

---

**Status**: Ready for implementation
**Next Steps**: Create skeleton header files with TODOs
