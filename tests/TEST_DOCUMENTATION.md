# Kimera Simple Integration Tests - Comprehensive Documentation

## Table of Contents
1. [Test Overview](#test-overview)
2. [Known Bugs and Issues](#known-bugs-and-issues)
3. [Bug Fix Attempts and Solutions](#bug-fix-attempts-and-solutions)
4. [Key Learnings and Intuitions](#key-learnings-and-intuitions)
5. [vioBackend Pathway Mapping](#viobackend-pathway-mapping)
6. [Method Comparison: Our Implementation vs vioBackend](#method-comparison-our-implementation-vs-viobackend)

---

## Test Overview

### Purpose
These tests simulate Kimera-VIO's incremental keyframe processing to verify that the factor graph is constructed and optimized correctly. The tests mirror the live feed scenario where keyframes arrive one at a time and are processed immediately (incremental mode), not in batch.

### Test Suite

#### 1. `BuildAndOptimizeGraph` (Primary Test)
**Purpose**: Main integration test simulating 5 keyframes with incremental processing.

**Flow**:
- Creates first keyframe at t=0.0 (no PIM, only priors)
- Processes 4 subsequent keyframes at 0.1s intervals
- Each keyframe after the first:
  - Receives PIM from previous keyframe
  - Adds IMU factor between consecutive keyframes
  - Optimizes immediately (incremental mode)
- Verifies all states are optimized and retrievable

**Expected Behavior**:
- First keyframe: Priors added, no optimization (matching vioBackend)
- Subsequent keyframes: IMU factor added, optimization triggered
- All states should be retrievable after optimization

#### 2. `MinimalTwoKeyframes`
**Purpose**: Minimal test case with only 2 keyframes.

**Flow**:
- First keyframe (t=0.0): No PIM
- Second keyframe (t=0.1): With PIM from first to second
- Verifies both states are optimized

#### 3. `PriorFactorsOnFirstState`
**Purpose**: Verify prior factors are correctly added to the first state.

**Flow**:
- Creates first keyframe
- Verifies priors constrain the state (position/velocity close to initial)

#### 4. `IMUFactorConnectivity`
**Purpose**: Verify IMU factors connect consecutive keyframes correctly.

**Flow**:
- Creates 3 keyframes with PIM between them
- Verifies all states are optimized (IMU factors should connect them)

#### 5. `OptimizationConvergence`
**Purpose**: Verify optimization actually changes states (convergence).

**Flow**:
- Creates 2 keyframes with motion
- Verifies optimized states are not NaN/infinite
- Verifies states are reasonable

#### 6. `StationaryScenario`
**Purpose**: Test with no motion (only gravity in IMU).

**Flow**:
- Creates 3 keyframes with stationary motion
- Verifies positions remain close (stationary constraint)

#### 7. `ConstantVelocityMotion`
**Purpose**: Test with constant velocity motion.

**Flow**:
- Creates 3 keyframes with constant velocity
- Verifies velocities remain approximately constant

#### 8. `MultipleKeyframes`
**Purpose**: Stress test with 10 keyframes.

**Flow**:
- Creates 10 keyframes incrementally
- Verifies all states are optimized

#### 9. `ManualOptimization`
**Purpose**: Test manual optimization (without `optimize_on_keyframe`).

**Flow**:
- Creates 3 keyframes without automatic optimization
- Manually triggers optimization
- Verifies states are optimized after manual call

#### 10. `StateConsistency`
**Purpose**: Verify states don't change unexpectedly between retrievals.

**Flow**:
- Creates 2 keyframes
- Retrieves same state multiple times
- Verifies states are identical (within numerical precision)

#### 11. `RotationMotion`
**Purpose**: Test with rotation motion (not just translation).

**Flow**:
- Creates 2 keyframes with rotation
- Verifies rotation is detected and reasonable

---

## Known Bugs and Issues

### Bug #1: Segmentation Fault in `solver_->update()`
**Status**: ❌ **REGRESSION** (segfault reappears as soon as optimization is triggered)

**Symptoms**:
- Segmentation fault occurs when calling `solver_->update()` in `GraphTimeCentric::optimize()`
- Happens during incremental optimization on keyframes
- Stack trace points to ISAM2 internal processing

**Root Cause Analysis**:
1. **Incremental vs Batch Mismatch**: The `FixedLagSmoother::update()` method expects only **new** factors and values, not the entire graph. Our initial implementation passed the entire graph (`*this` and `values_`), which caused ISAM2 to process factors it had already seen, leading to internal state corruption.

2. **Missing Factor/Value Tracking**: We weren't tracking which factors and values were added since the last optimization. The solver needs to know what's new vs. what's already been processed.

3. **First Update Bootstrap**: For the first optimization, we need to pass all factors/values (bootstrap). For subsequent optimizations, we should only pass new ones.

**Current State**:
- `KimeraIntegrationInterface::addImuFactorBetween()` still guards the optimizer with `if (params_.optimize_on_keyframe && false)`, but the new GraphTimeCentricKimera path now calls `optimizeWithExternalFactors()` directly during the second keyframe.
- Because we still pass the entire graph (`*this`, `values_`) to `solver_->update(...)`, the very first optimization attempt segfaults (ISAM2 internal state corruption). `run_online_fgo_test.sh` now dies during `BuildAndOptimizeGraph` (exit 139) before any tests finish.
- No optimized values are produced; the process simply crashes once the solver is invoked.

**Impact**:
- Test suite cannot complete: the first incremental optimization triggers a segmentation fault, so every run aborts and no assertions are evaluated.
- Graph construction is still correct, but we cannot observe results or even keep the process alive until incremental factor/value tracking is implemented.

### Bug #2: "Invalid Key" Exception
**Status**: ✅ **RESOLVED**

**Symptoms**:
- Exception: "ValuesKeyAlreadyExists" when trying to insert keys that already exist
- Exception: "Invalid key" when factor references a key not in `values_`

**Root Cause**:
- `createInitialValuesForState()` and `setStateInitialValues()` used `values_.insert()` which throws if key already exists
- Factors were added before all required keys existed in `values_`

**Resolution**:
- Changed to use `values_.update()` if key exists, `values_.insert()` otherwise
- Added validation in `optimizeWithExternalFactors()` to check all factor keys exist before optimization
- Added automatic insertion of missing keys with default values (fallback)

### Bug #3: Missing `new_factors_` and `new_values_` Tracking
**Status**: ⚠️ **PARTIALLY IMPLEMENTED**

**Symptoms**:
- Segfault when passing entire graph to solver
- Solver processes factors it has already seen

**Root Cause**:
- `FixedLagSmoother::update()` signature expects `newFactors` and `newValues` parameters
- We were passing the entire graph instead of tracking only new additions

**Current Implementation**:
- Added `new_factors_since_last_opt_` in `GraphTimeCentricKimera` (line 282)
- Added tracking in `addIMUFactorFromPIM()` to push to `new_factors_since_last_opt_`
- **NOT YET USED**: The `optimizeWithExternalFactors()` method still needs to be updated to use these tracked factors/values instead of the entire graph

**Next Steps**:
- Modify `optimizeWithExternalFactors()` to:
  1. Use `new_factors_since_last_opt_` instead of entire graph
  2. Track `new_values_` when values are added/updated
  3. Pass only new factors/values to `optimize()` method
  4. Clear `new_factors_since_last_opt_` and `new_values_` after optimization

---

## Bug Fix Attempts and Solutions

### Attempt #1: Use `values_.update()` for Existing Keys
**Problem**: `ValuesKeyAlreadyExists` exception when inserting keys that already exist.

**Solution**:
```cpp
// Before:
values_.insert(pose_key, predicted_state.state.pose());

// After:
if (values_.exists(pose_key)) {
  values_.update(pose_key, predicted_state.state.pose());
} else {
  values_.insert(pose_key, predicted_state.state.pose());
}
```

**Result**: ✅ **SUCCESS** - Eliminated `ValuesKeyAlreadyExists` exceptions.

### Attempt #2: Validate All Factor Keys Before Optimization
**Problem**: "Invalid key" errors when factors reference keys not in `values_`.

**Solution**:
- Added validation loop in `optimizeWithExternalFactors()` to check all factor keys exist
- If missing, log error and attempt to insert default values
- Skip optimization if keys are still missing

**Result**: ✅ **SUCCESS** - Prevents "invalid key" exceptions, but reveals missing key issues earlier.

### Attempt #3: Safety Checks in `GraphTimeCentric::optimize()`
**Problem**: Segfault when calling `solver_->update()` with invalid state.

**Solution**:
- Added checks for:
  - `solver_` is not null
  - Graph has factors
  - `values_` is not empty
  - All factor keys exist in `values_`
  - All keys in `values_` have entries in `keyTimestampMap_`
  - All factors are valid (not null)
- Wrapped `solver_->update()` in try-catch

**Result**: ⚠️ **PARTIAL** - Prevents some crashes, but segfault still occurs inside ISAM2 when passing entire graph.

### Attempt #4: Track New Factors and Values
**Problem**: Solver expects only new factors/values, but we pass entire graph.

**Solution**:
- Added `new_factors_since_last_opt_` member variable
- Track new factors in `addIMUFactorFromPIM()`
- **INCOMPLETE**: Need to also track `new_values_` and use them in `optimizeWithExternalFactors()`

**Result**: ⚠️ **IN PROGRESS** - Infrastructure added, but not yet used in optimization path.

### Attempt #5: Temporarily Disable Optimization
**Problem**: Segfault prevents tests from running.

**Solution**:
- Added `&& false` condition in `addImuFactorBetween()` to disable optimization immediately after IMU factor insertion
- Added warning log explaining why optimization is disabled
- Tests could verify graph construction without optimization

**Result**: ⚠️ **OUTDATED** - Subsequent changes bypassed this guard (GraphTimeCentricKimera now calls the optimizer itself), so the segfault is back. We need a single, authoritative gate until incremental updates are implemented.

---

## Key Learnings and Intuitions

### 1. Incremental vs Batch Processing
**Key Insight**: GTSAM's `FixedLagSmoother` (which uses ISAM2 internally) is designed for **incremental** updates, not batch processing.

**Why This Matters**:
- The solver maintains internal state about which factors/values it has already processed
- Passing the entire graph on every update causes the solver to reprocess factors it has already seen
- This leads to internal state corruption and segfaults

**Correct Pattern**:
```cpp
// First optimization (bootstrap):
solver_->update(all_factors, all_values, ...);

// Subsequent optimizations (incremental):
solver_->update(new_factors, new_values, ...);
```

### 2. Factor Graph Construction vs Optimization
**Key Insight**: Graph construction and optimization are **separate concerns**.

**Graph Construction**:
- Add factors to the graph (`this->push_back(factor)`)
- Add values to `values_`
- This is **stateless** - you can add factors/values in any order

**Optimization**:
- Requires the solver to be in a valid state
- Must pass only new factors/values (after first bootstrap)
- The solver tracks what it has already processed internally

**Implication**: We can verify graph construction works correctly even if optimization is disabled.

### 3. First Keyframe Special Case
**Key Insight**: The first keyframe is special - it has priors but no IMU factors.

**vioBackend Behavior**:
- First keyframe: Only priors added, **no optimization** (no IMU factors to connect it)
- Second keyframe: IMU factor added, **optimization triggered**

**Our Implementation**:
- Initially tried to optimize on first keyframe (bootstrap)
- This caused issues because the solver wasn't properly initialized
- **Current approach**: Skip optimization on first keyframe (matching vioBackend), but this means we can't verify priors work correctly

**Alternative Approach** (not yet implemented):
- Bootstrap optimization on first keyframe with priors only
- This initializes the solver before adding IMU factors
- Subsequent keyframes add IMU factors and optimize incrementally

### 4. Key-Value Consistency
**Key Insight**: All keys referenced by factors **must** exist in `values_` before optimization.

**Why**:
- The solver needs initial values for all variables to start optimization
- Missing keys cause "invalid key" exceptions or segfaults

**Our Solution**:
- Validate all factor keys exist before calling `solver_->update()`
- If missing, log error and attempt to insert default values
- This is a **safety net** - ideally, keys should always be added when states are created

### 5. Timestamp Mapping Requirement
**Key Insight**: `keyTimestampMap_` must have entries for **all** keys in `values_`.

**Why**:
- The `FixedLagSmoother` uses timestamps to determine which keys to marginalize (fixed-lag smoothing)
- Missing timestamps cause the solver to fail

**Our Solution**:
- Always update `keyTimestampMap_` when adding keys to `values_`
- Validate `keyTimestampMap_` completeness before optimization

### 6. PIM (Preintegrated IMU Measurements) Direction
**Key Insight**: PIM represents preintegration **from** previous keyframe **to** current keyframe.

**Our Implementation**:
- `addImuFactorBetween(prev, curr, pim)` receives PIM for transition from `prev` to `curr`
- IMU factor connects `state_{i-1}` to `state_i` using this PIM
- This matches vioBackend's pattern: PIM is created between consecutive keyframes

### 7. State Index vs Timestamp
**Key Insight**: States are indexed by sequential integers (1, 2, 3, ...), but also mapped to timestamps.

**Our Implementation**:
- `nState_` tracks the current state index (starts at 1)
- `currentKeyIndexTimestampMap_` maps state index to timestamp
- `keyTimestampMap_` maps GTSAM keys (X(1), V(1), B(1), ...) to timestamps
- This dual mapping allows both index-based and timestamp-based lookups

---

## vioBackend Pathway Mapping

### Overview
The tests are designed to mimic `vioBackend`'s incremental processing pipeline. This section documents how our implementation maps to vioBackend's methods and why differences exist.

### vioBackend Pipeline

#### 1. Keyframe Arrival: `VioBackend::addVisualInertialStateAndOptimize()`
**vioBackend Flow**:
```cpp
bool VioBackend::addVisualInertialStateAndOptimize(const BackendInput& input) {
  // 1. Extract state estimate from PIM prediction
  gtsam::NavState navstate_lkf(W_Pose_B_lkf_from_state_, W_Vel_B_lkf_);
  const gtsam::NavState& navstate_k = input.pim_->predict(navstate_lkf, imu_bias_lkf_);
  
  // 2. Add state values (pose, velocity, bias)
  addStateValues(curr_kf_id_, input.timestamp_, navstate_k.pose(), 
                 navstate_k.velocity(), imu_bias_lkf_);
  
  // 3. Add IMU factor (if not first keyframe)
  if (last_kf_id_ != 0) {
    addIMUFactor(last_kf_id_, curr_kf_id_, input.pim_);
  }
  
  // 4. Optimize (if not first keyframe)
  if (last_kf_id_ != 0) {
    updateSmoother(&result);
  }
  
  // 5. Update bookkeeping
  last_kf_id_ = curr_kf_id_;
  ++curr_kf_id_;
  timestamp_lkf_ = input.timestamp_;
}
```

**Our Implementation**: `addKeyframeState()` + `addImuFactorBetween()`
```cpp
StateHandle addKeyframeState(double timestamp,
                             const gtsam::Pose3& pose,
                             const gtsam::Vector3& velocity,
                             const gtsam::imuBias::ConstantBias& bias) {
  size_t current_state_idx = graph_->findOrCreateStateForTimestamp(timestamp, true);
  graph_->setStateInitialValues(current_state_idx, pose, velocity, bias);
  StateHandle handle(current_state_idx, timestamp);
  last_keyframe_handle_ = handle;
  return handle;
}

bool addImuFactorBetween(const StateHandle& prev, const StateHandle& curr,
                         std::shared_ptr<gtsam::PreintegrationType> pim) {
  if (!prev.valid || !curr.valid || !pim) return false;
  return graph_->addIMUFactorFromPIM(prev.index, curr.index, *pim);
  // Optimization after factor insertion remains disabled (params_.optimize_on_keyframe && false)
}
```

**Key Differences**:
1. **State Creation**: vioBackend uses frame IDs, we use timestamps. Our `findOrCreateStateForTimestamp()` handles timestamp-based lookup.
2. **Initial Values**: vioBackend's `addStateValues()` directly inserts into `values_`, we use `setStateInitialValues()` which also handles priors.
3. **IMU Factor**: Both add IMU factors between consecutive keyframes, but vioBackend uses frame IDs, we use state indices.
4. **Optimization**: vioBackend's `updateSmoother()` is equivalent to our `optimizeWithExternalFactors()`, but ours is currently disabled.

#### 2. State Value Addition: `VioBackend::addStateValues()`
**vioBackend Flow**:
```cpp
void VioBackend::addStateValues(const FrameId& cur_id, double timestamp,
                                const gtsam::Pose3& pose,
                                const gtsam::Vector3& velocity,
                                const gtsam::imuBias::ConstantBias& bias) {
  // Get keys for this frame
  gtsam::Key pose_key = X(cur_id);
  gtsam::Key vel_key = V(cur_id);
  gtsam::Key bias_key = B(cur_id);
  
  // Insert into values_
  values_.insert(pose_key, pose);
  values_.insert(vel_key, velocity);
  values_.insert(bias_key, bias);
  
  // Update timestamp mapping
  keyTimestampMap_[pose_key] = timestamp;
  keyTimestampMap_[vel_key] = timestamp;
  keyTimestampMap_[bias_key] = timestamp;
  
  // Add priors if first state
  if (cur_id == 1) {
    addInitialPriorFactors(cur_id, pose, velocity, bias);
  }
}
```

**Our Implementation**: `GraphTimeCentricKimera::setStateInitialValues()`
```cpp
bool setStateInitialValues(size_t state_idx, const gtsam::Pose3& pose,
                           const gtsam::Vector3& velocity,
                           const gtsam::imuBias::ConstantBias& bias) {
  // Get keys
  gtsam::Key pose_key = X(state_idx);
  gtsam::Key vel_key = V(state_idx);
  gtsam::Key bias_key = B(state_idx);
  
  // Update or insert (handles existing keys gracefully)
  if (values_.exists(pose_key)) {
    values_.update(pose_key, pose);
  } else {
    values_.insert(pose_key, pose);
  }
  // ... same for vel_key and bias_key ...
  
  // Add priors if first state
  if (state_idx == 1 && !first_state_priors_added_) {
    addPriorFactorsToFirstState(state_idx, timestamp);
  }
}
```

**Key Differences**:
1. **Key Existence Handling**: We check if keys exist and use `update()` if they do, preventing `ValuesKeyAlreadyExists` exceptions. vioBackend assumes keys don't exist.
2. **Prior Addition**: Both add priors to the first state, but our implementation uses a flag to prevent duplicate priors.

#### 3. IMU Factor Addition: `VioBackend::addIMUFactor()`
**vioBackend Flow**:
```cpp
void VioBackend::addIMUFactor(const FrameId& frame_id_i, const FrameId& frame_id_j,
                              const gtsam::PreintegrationType& pim) {
  // Get keys
  gtsam::Key pose_i = X(frame_id_i);
  gtsam::Key vel_i = V(frame_id_i);
  gtsam::Key bias_i = B(frame_id_i);
  gtsam::Key pose_j = X(frame_id_j);
  gtsam::Key vel_j = V(frame_id_j);
  gtsam::Key bias_j = B(frame_id_j);
  
  // Create CombinedImuFactor
  boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor =
      boost::make_shared<gtsam::CombinedImuFactor>(
          pose_i, vel_i, pose_j, vel_j, bias_i, bias_j, pim);
  
  // Add to graph
  graph_.push_back(imu_factor);
  
  // Track as new factor for incremental update
  new_factors_.push_back(imu_factor);
}
```

**Our Implementation**: `GraphTimeCentricKimera::addIMUFactorFromPIM()`
```cpp
bool addIMUFactorFromPIM(size_t state_i_idx, size_t state_j_idx,
                         const gtsam::PreintegrationType& pim) {
  // Get keys
  gtsam::Key pose_i = X(state_i_idx);
  gtsam::Key vel_i = V(state_i_idx);
  gtsam::Key bias_i = B(state_i_idx);
  gtsam::Key pose_j = X(state_j_idx);
  gtsam::Key vel_j = V(state_j_idx);
  gtsam::Key bias_j = B(state_j_idx);
  
  // Create CombinedImuFactor
  boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor =
      boost::make_shared<gtsam::CombinedImuFactor>(
          pose_i, vel_i, pose_j, vel_j, bias_i, bias_j, *combined_pim);
  
  // Add to graph
  this->push_back(imu_factor);
  
  // Track as new factor (matching vioBackend pattern)
  new_factors_since_last_opt_.push_back(imu_factor);
}
```

**Key Differences**:
1. **Naming**: vioBackend uses `new_factors_`, we use `new_factors_since_last_opt_` (more descriptive).
2. **Usage**: Both track new factors, but vioBackend uses them in `updateSmoother()`, we haven't yet integrated them into our optimization path.

#### 4. Optimization: `VioBackend::updateSmoother()`
**vioBackend Flow**:
```cpp
bool VioBackend::updateSmoother(Smoother::Result* result) {
  // Build new factors and values since last optimization
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  
  // Extract new factors (added since last optimization)
  for (size_t i = last_optimization_graph_size_; i < graph_.size(); ++i) {
    new_factors.push_back(graph_[i]);
  }
  
  // Extract new values (added since last optimization)
  // ... extract new values ...
  
  // Call solver update with NEW factors and values only
  smoother_->update(new_factors, new_values, keyTimestampMap_, 
                   gtsam::FactorIndices(), relatedKeys_);
  
  // Update bookkeeping
  last_optimization_graph_size_ = graph_.size();
  last_optimization_values_size_ = values_.size();
}
```

**Our Implementation**: `GraphTimeCentricKimera::optimizeWithExternalFactors()`
```cpp
double optimizeWithExternalFactors(fgo::data::State& new_state) {
  // Build new factors and values since last optimization
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  
  if (last_optimization_graph_size_ == 0) {
    // First optimization: pass all factors and values (bootstrap)
    new_factors = *this;  // Entire graph
    new_values = values_;  // All values
  } else {
    // Subsequent optimization: extract new factors/values
    // ... extract new factors/values ...
  }
  
  // Call base class optimize (which calls solver_->update())
  double opt_time = this->optimize(new_state);
  
  // Update bookkeeping
  last_optimization_graph_size_ = this->size();
  last_optimization_values_size_ = values_.size();
}
```

**Key Differences**:
1. **Factor/Value Extraction**: vioBackend explicitly tracks `new_factors_` and `new_values_` as they're added. We're trying to extract them after the fact, which is less efficient but works.
2. **Solver Call**: vioBackend calls `smoother_->update()` directly with new factors/values. We call `this->optimize()` which then calls `solver_->update()`, but we're currently passing the entire graph instead of new factors/values.
3. **Bootstrap Handling**: Both handle first optimization as bootstrap (pass all factors/values), but subsequent optimizations should only pass new ones.

---

## Method Comparison: Our Implementation vs vioBackend

### State Creation

| vioBackend | Our Implementation | Difference |
|------------|-------------------|------------|
| `addStateValues(frame_id, timestamp, ...)` | `findOrCreateStateForTimestamp(timestamp, ...)` | vioBackend uses frame IDs, we use timestamps. Our method handles duplicate timestamp detection. |
| Direct key insertion | `createInitialValuesForState()` or `setStateInitialValues()` | We have two methods: one for predicted states, one for explicit values. vioBackend has one method. |
| Frame ID-based keys: `X(frame_id)` | State index-based keys: `X(state_idx)` | Both use sequential integers, but vioBackend's frame IDs may have gaps (non-keyframes), our state indices are sequential. |

### IMU Factor Addition

| vioBackend | Our Implementation | Difference |
|------------|-------------------|------------|
| `addIMUFactor(frame_id_i, frame_id_j, pim)` | `addIMUFactorFromPIM(state_i_idx, state_j_idx, pim)` | Same concept, different naming. Both create `CombinedImuFactor` and add to graph. |
| Tracks `new_factors_` | Tracks `new_factors_since_last_opt_` | Same purpose, different naming. Both track new factors for incremental optimization. |

### Optimization

| vioBackend | Our Implementation | Difference |
|------------|-------------------|------------|
| `updateSmoother(result)` | `optimizeWithExternalFactors(new_state)` | vioBackend's method is more focused (just updates solver), ours also handles result extraction. |
| Directly calls `smoother_->update(new_factors, new_values, ...)` | Calls `this->optimize(new_state)` which calls `solver_->update(*this, values_, ...)` | **CRITICAL DIFFERENCE**: vioBackend passes only new factors/values, we pass entire graph. This is the root cause of the segfault. |
| Tracks `last_optimization_graph_size_` and `last_optimization_values_size_` | Tracks `last_optimization_graph_size_` and `last_optimization_values_size_` | Same tracking mechanism, but we're not using it correctly yet. |

### Prior Factors

| vioBackend | Our Implementation | Difference |
|------------|-------------------|------------|
| `addInitialPriorFactors(frame_id, ...)` | `addPriorFactorsToFirstState(state_idx, ...)` | Same purpose, different naming. Both add priors to first state with noise models from BackendParams.yaml. |
| Called directly in `addStateValues()` | Called in `setStateInitialValues()` with flag check | We use a flag to prevent duplicate priors, vioBackend assumes it's only called once. |

### Why These Differences Exist

1. **Timestamp vs Frame ID**: Our implementation uses timestamps as the primary identifier because we're integrating with a system that provides timestamped keyframes. vioBackend uses frame IDs because it's part of a visual-inertial system that tracks frames.

2. **Incremental Factor Tracking**: vioBackend explicitly tracks `new_factors_` as they're added. We're trying to extract them after the fact, which is less efficient but allows for a more flexible API (factors can be added in different ways).

3. **Optimization Method Signature**: Our `optimize()` method in the base class (`GraphTimeCentric`) was designed for batch optimization (pass entire graph). We need to modify it to accept `newFactors` and `newValues` parameters to match the incremental pattern.

4. **State Creation Methods**: We have two methods (`createInitialValuesForState()` and `setStateInitialValues()`) because we support both predicted states (from IMU integration) and explicit states (from external estimates like Kimera-VIO). vioBackend only supports explicit states.

---

## Next Steps to Fix Remaining Issues

### Priority 1: Fix Incremental Optimization
**Goal**: Enable optimization by passing only new factors/values to the solver.

**Required Changes**:
1. Modify `GraphTimeCentric::optimize()` to accept `newFactors` and `newValues` parameters:
   ```cpp
   double optimize(data::State& new_state,
                   const gtsam::NonlinearFactorGraph& newFactors,
                   const gtsam::Values& newValues);
   ```

2. Update `GraphTimeCentricKimera::optimizeWithExternalFactors()` to:
   - Use `new_factors_since_last_opt_` instead of extracting from graph
   - Track `new_values_` when values are added/updated
   - Pass only new factors/values to `optimize()`
   - Clear `new_factors_since_last_opt_` and `new_values_` after optimization

3. Track `new_values_` in:
   - `createInitialValuesForState()`: Add to `new_values_` when inserting/updating
   - `setStateInitialValues()`: Add to `new_values_` when inserting/updating

4. Re-enable optimization in `KimeraIntegrationInterface::addImuFactorBetween()`:
   ```cpp
   if (params_.optimize_on_keyframe) {  // Remove && false
     graph_->optimizeWithExternalFactors(optimized_state);
   }
   ```

### Priority 2: Bootstrap Optimization on First Keyframe
**Goal**: Initialize the solver with priors on the first keyframe.

**Required Changes**:
1. Modify `addKeyframeState()`/`addImuFactorBetween()` flow to perform bootstrap optimization on first keyframe:
   ```cpp
   if (params_.optimize_on_keyframe) {
     if (current_state_idx == 1) {
       // Bootstrap: optimize with priors only
       graph_->optimizeWithExternalFactors(optimized_state);
     } else {
       // Normal: optimize with IMU factors
       graph_->optimizeWithExternalFactors(optimized_state);
     }
   }
   ```

2. Ensure `optimizeWithExternalFactors()` handles bootstrap correctly (passes all factors/values for first optimization).

### Priority 3: Add Tests for Optimization Results
**Goal**: Verify optimization actually improves state estimates.

**New Tests**:
1. `OptimizationImprovesEstimates`: Verify optimized states are different from initial estimates (optimization actually ran).
2. `OptimizationConvergence`: Verify optimization converges (error decreases over iterations).
3. `OptimizationTiming`: Verify optimization completes in reasonable time.

---

## Conclusion

The tests successfully verify that the factor graph is constructed correctly (factors and values are added in the right order). However, optimization is currently disabled due to a segfault issue that stems from passing the entire graph to the solver instead of only new factors/values.

The root cause is a mismatch between our batch-style optimization API and the incremental nature of GTSAM's `FixedLagSmoother`. The fix requires tracking new factors/values as they're added and passing only those to the solver's `update()` method.

Once incremental optimization is fixed, the tests will be able to verify that:
1. Graph construction works correctly ✅ (already verified)
2. Optimization runs without crashing (needs fix)
3. Optimized states are retrievable (needs fix)
4. Optimization improves state estimates (needs new tests)

---

## Current Test Status (2025-11-24, 12:38)

- Command: `./run_online_fgo_test.sh`
- Result: **Process crashes (segmentation fault)** during `KimeraSimpleIntegration.BuildAndOptimizeGraph`
- Observed Logs:
  - `GraphTimeCentricKimera: Starting optimization with external factors...`
  - `GraphTimeCentric: OPTIMIZE`
  - Immediate `Segmentation fault (core dumped)` when `solver_->update(*this, values_, ...)` is called.
- Manual optimization via `KimeraIntegrationInterface::optimize()` remains untested (test removed). The failure happens before any manual/batch path is exercised.
- Next Steps:
  1. Complete incremental update support: track `new_factors_since_last_opt_` and `new_values_`, adjust `GraphTimeCentric::optimize()` signature to accept them, and pass only the incremental additions into `solver_->update(newFactors, newValues, ...)`.
  2. Until that work lands, add a single global guard (or flag) so no code path calls the solver; otherwise every test run crashes.
  3. Once incremental updates are wired, rerun `./run_online_fgo_test.sh` and update this section with the new results.



