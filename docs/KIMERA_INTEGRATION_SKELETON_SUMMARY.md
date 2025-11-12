# Kimera VIO Integration - Skeleton Files Created

## Overview
This document summarizes the skeleton header files created for integrating Kimera VIO with online_fgo_core's GraphTimeCentric framework. All files include comprehensive TODOs and are ready for implementation.

## Files Created/Updated

### 1. online_fgo_core Package

#### GraphTimeCentricKimera.h
**Location**: `online_fgo_core/include/online_fgo_core/graph/GraphTimeCentricKimera.h`

**Status**: UPDATED (existing file enhanced with IMU-first functionality)

**Key Features**:
- Extends `GraphTimeCentric` with Kimera-specific functionality
- **State Management** (Core IMU-first feature):
  - `findOrCreateStateForTimestamp()` - Create states at arbitrary timestamps
  - `hasStateAtTimestamp()` - Check if state exists
  - `getStateIndexAtTimestamp()` - Lookup state by timestamp
  - `getAllStateTimestamps()` - Get all timestamps

- **IMU Measurement Handling**:
  - `addIMUMeasurements()` - Buffer IMU data
  - `constructFactorGraphFromTimestamps()` - Build graph from timestamp list
  - `addIMUFactorBetweenStates()` - Create CombinedImuFactor

- **GP Motion Priors**:
  - `addGPMotionPriorsForStates()` - Batch add GP priors
  - `addGPMotionPriorBetweenStates()` - Single GP prior

- **External Factor Interface** (for future use):
  - `addExternalFactor()` - Add factors from Kimera

- **Optimization**:
  - `optimizeWithExternalFactors()` - Run optimization

- **Result Retrieval**:
  - `getOptimizedPose()`, `getOptimizedVelocity()`, `getOptimizedBias()`
  - `getStateCovariance()`

- **Smart Factor Management** (future Phase 3):
  - `addLandmarkToGraph()`, `cleanCheiralityLandmarks()`, etc.

**Parameters**: `GraphTimeCentricKimeraParams` struct with all configuration

**TODOs**: 27 TODO items covering all implementation details

---

#### KimeraIntegrationInterface.h
**Location**: `online_fgo_core/include/online_fgo_core/integration/KimeraIntegrationInterface.h`

**Status**: NEW FILE

**Purpose**: Clean interface for Kimera adapter to interact with GraphTimeCentric

**Key Features**:
- **Initialization**:
  - `initialize()` - Setup with parameters
  - `isInitialized()` - Check readiness

- **State Management**:
  - `createStateAtTimestamp()` - Create state at timestamp
  - `getStateAtTimestamp()` - Retrieve existing state
  - `getAllStateTimestamps()` - Get all timestamps

- **IMU Data Handling**:
  - `addIMUData()` - Single IMU measurement
  - `addIMUDataBatch()` - Multiple IMU measurements

- **Optimization**:
  - `optimize()` - Trigger optimization with full result
  - `optimizeAndGetLatestState()` - Convenience method

- **Result Retrieval**:
  - `getOptimizedState()`, `getOptimizedBias()`, `getStateCovariance()`
  - `getLatestOptimizedState()`, `getLatestOptimizedBias()`

**Data Structures**:
- `KimeraIntegrationParams` - Configuration parameters
- `StateHandle` - Handle to identify states
- `OptimizationResult` - Optimization output with diagnostics
- `FactorData` - Generic factor data (future use)

**Design Pattern**: Facade pattern hiding GraphTimeCentric complexity

**TODOs**: 15 TODO items

---

### 2. Kimera-VIO Package

#### GraphTimeCentricBackendAdapter.h
**Location**: `Kimera-VIO/include/kimera-vio/integration/GraphTimeCentricBackendAdapter.h`

**Status**: UPDATED (existing minimal file expanded)

**Purpose**: Adapter in Kimera VIO that translates VioBackend calls to GraphTimeCentric

**Key Features**:
- **Initialization**:
  - `initialize()` - With ApplicationInterface
  - `initializeStandalone()` - For standalone testing

- **State Management**:
  - `addKeyframeState()` - Create state at keyframe timestamp (2 overloads)
  - `addStateValues()` - Legacy interface

- **IMU Handling**:
  - `addIMUMeasurement()` - Single measurement
  - `addIMUMeasurements()` - Batch measurements
  - `addIMUTimestamps()` - Legacy interface
  - `preintegrateIMUBetweenStates()` - Explicit preintegration

- **Optimization**:
  - `optimizeGraph()` - Trigger optimization
  - `optimize()` - Legacy interface
  - `getLastOptimizationTime()` - Performance metric

- **Result Retrieval**:
  - `getStateAtTime()`, `getLatestState()`, `getLatestIMUBias()`
  - `getStateCovariance()`, `getLatestStateCovariance()`
  - `getLastResult()` - Returns gtsam::Values

- **Statistics**:
  - `getNumStates()`, `getNumBufferedIMU()`, `getStatistics()`

**Member Variables**:
- `integration_interface_` - Connection to online_fgo_core
- `imu_buffer_` - Buffered IMU measurements
- `keyframe_states_` - Map of keyframe timestamps to state handles
- Thread-safe with mutex protection

**Helper Methods**:
- `timestampToSeconds()` / `secondsToTimestamp()` - Timestamp conversion
- `createIntegrationParams()` - Parameter translation
- `findStateHandleNearTimestamp()` - State lookup

**TODOs**: 14 TODO items

---

### 3. Documentation

#### KIMERA_INTEGRATION_PLAN.md
**Location**: `online_fgo_core/docs/KIMERA_INTEGRATION_PLAN.md`

**Status**: NEW FILE

**Contents**:
- Comprehensive implementation plan
- Architecture and design principles
- Phase-by-phase implementation guide
- Testing strategy
- Parameter configuration
- Performance considerations
- Future extensions (smart factors, loop closure)

**Sections**:
1. Overview and Architecture
2. Phase 1: Build System & Scaffolding
3. Phase 2: Core Interfaces (IMU-Only)
4. Phase 3: Implementation Details
5. Phase 4: Testing Strategy
6. Phase 5: Future Extensions
7. Implementation Checklist
8. Key Design Decisions
9. Parameter Configuration
10. Error Handling
11. Performance Considerations

---

## Implementation Workflow

### Current Status: Skeleton Complete ✓

All header files are created with:
- ✅ Complete method signatures
- ✅ Comprehensive documentation
- ✅ TODO items for implementation
- ✅ Design rationale in comments
- ✅ Correct /include and /src separation

### Next Steps for Implementation:

1. **Create corresponding .cpp files** with TODO-driven implementation
2. **Update CMakeLists.txt** to add build flags and conditionally compile
3. **Implement core methods** following TODOs in order:
   - Start with `findOrCreateStateForTimestamp()`
   - Then IMU measurement buffering
   - Then factor creation
   - Finally optimization and result retrieval

4. **Write unit tests** for each component
5. **Integration testing** with IMU-only dataset

---

## Key Design Decisions Summary

### 1. IMU-First State Creation
- States created at arbitrary timestamps (not just IMU arrivals)
- `findOrCreateStateForTimestamp()` as core method
- Tolerance-based timestamp matching (default 1ms)

### 2. Adapter Pattern
- Clean separation between Kimera and online_fgo_core
- `KimeraIntegrationInterface` provides clean facade
- `GraphTimeCentricBackendAdapter` translates Kimera calls

### 3. Timestamp Indexing
- All states indexed by timestamp in `keyTimestampMap_`
- Efficient lookup via `currentKeyIndexTimestampMap_`
- Supports non-sequential state creation

### 4. Thread Safety
- Mutexes in adapter for buffer access
- Single-threaded graph operations
- Caller responsible for VioBackend-level synchronization

### 5. Build System
- Compile-time flags: `ENABLE_KIMERA_INTEGRATION` (online_fgo_core)
- Compile-time flags: `ENABLE_GRAPH_TIME_CENTRIC_ADAPTER` (Kimera-VIO)
- Runtime toggle in VioBackend parameters

---

## TODO Summary by Component

### GraphTimeCentricKimera (27 TODOs)
- State management: 7 TODOs
- IMU handling: 5 TODOs
- GP priors: 4 TODOs
- External factors: 3 TODOs
- Optimization: 2 TODOs
- Result retrieval: 4 TODOs
- Helper methods: 2 TODOs

### KimeraIntegrationInterface (15 TODOs)
- Initialization: 3 TODOs
- State management: 3 TODOs
- IMU handling: 2 TODOs
- Optimization: 2 TODOs
- Result retrieval: 4 TODOs
- Helper: 1 TODO

### GraphTimeCentricBackendAdapter (14 TODOs)
- Initialization: 2 TODOs
- State management: 2 TODOs
- IMU handling: 4 TODOs
- Optimization: 2 TODOs
- Result retrieval: 1 TODO
- Helpers: 3 TODOs

**Total: 56 TODO items** providing clear implementation guidance

---

## Integration with Existing Code

### online_fgo_core Integration Points
1. `GraphBase` - Base class provides:
   - `values_`, `keyTimestampMap_`, `currentKeyIndexTimestampMap_`
   - `preIntegratorParams_`
   - `solver_` (FixedLagSmoother)
   - `currentPredictedBuffer_`
   - Helper methods for GP factors

2. `GraphTimeCentric` - Parent class provides:
   - `constructFactorGraphOnTime()` - Reference implementation
   - IMU factor creation patterns
   - Optimization flow

### Kimera-VIO Integration Points
1. `VioBackend` - Needs modifications:
   - Add `use_graph_time_centric` parameter
   - Instantiate adapter when enabled
   - Route IMU/keyframe calls through adapter

2. `BackendParams` - Add:
   - `bool use_graph_time_centric`
   - `std::string graph_config_path`

---

## File Structure Summary

```
online_fgo_core/
├── docs/
│   └── KIMERA_INTEGRATION_PLAN.md          ✅ NEW
├── include/online_fgo_core/
│   ├── graph/
│   │   └── GraphTimeCentricKimera.h        ✅ UPDATED
│   └── integration/
│       └── KimeraIntegrationInterface.h    ✅ NEW
└── src/
    ├── graph/
    │   └── GraphTimeCentricKimera.cpp      ❌ TODO: CREATE
    └── integration/
        └── KimeraIntegrationInterface.cpp  ❌ TODO: CREATE

Kimera-VIO/
├── include/kimera-vio/
│   └── integration/
│       └── GraphTimeCentricBackendAdapter.h ✅ UPDATED
└── src/
    └── integration/
        └── GraphTimeCentricBackendAdapter.cpp ❌ TODO: CREATE
```

---

## Next Actions for AI Implementation

When ready to implement .cpp files:

1. **Start with GraphTimeCentricKimera.cpp**:
   ```cpp
   // Implement in this order:
   1. Constructor and initializeKimeraSupport()
   2. findOrCreateStateForTimestamp() and helpers
   3. addIMUMeasurements() and IMU buffering
   4. constructFactorGraphFromTimestamps()
   5. addGPMotionPriorsForStates()
   6. optimizeWithExternalFactors()
   7. Result retrieval methods
   ```

2. **Then KimeraIntegrationInterface.cpp**:
   - Facade implementation wrapping GraphTimeCentricKimera
   - Parameter conversion helpers
   - IMU data conversion

3. **Then GraphTimeCentricBackendAdapter.cpp**:
   - Kimera timestamp conversion
   - IMU buffer management
   - State handle tracking
   - Result translation back to Kimera format

4. **Update CMakeLists.txt in both packages**:
   - Add build options
   - Conditional compilation
   - Link dependencies

5. **Write unit tests**:
   - State creation tests
   - IMU factor tests
   - Optimization tests

---

## References

- **todo.md**: Original detailed task breakdown
- **GraphTimeCentric.cpp**: Reference implementation for IMU factors and GP priors
- **GraphBase.h**: Base class interface and utilities
- **VioBackend.cpp**: Kimera's native backend (for comparison)

---

**Status**: All skeleton files complete and ready for implementation
**Last Updated**: 2024-10-31
**AI Implementation Ready**: Yes - all TODOs are clear and actionable
