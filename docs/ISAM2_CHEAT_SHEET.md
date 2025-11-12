# ISAM2 Quick Cheat Sheet

## One-Minute Summary

**ISAM2** = Fast incremental optimizer for factor graphs
- Only updates what changed (not full re-optimization)
- Maintains Bayes Tree for efficiency
- Perfect for real-time SLAM/navigation

**Your Usage**: Fixed-lag smoother for Kimera integration
- Optimizes last `smootherLag` seconds of states
- Marginalizes old states automatically
- Constant-time performance

---

## Essential Code Patterns

### Setup
```cpp
// Configure ISAM2
gtsam::ISAM2Params params;
params.relinearizeThreshold = 0.1;  // Lower = more accurate
params.relinearizeSkip = 10;        // How often to check
params.factorization = gtsam::ISAM2Params::CHOLESKY;

// Create smoother with time window
double smootherLag = 3.0;  // Keep 3 seconds
IncrementalFixedLagSmoother smoother(smootherLag, params);
```

### Update
```cpp
// Build factor graph
NonlinearFactorGraph factors;
Values initialValues;
KeyTimestampMap timestamps;

// Add your factors (IMU, GP priors, etc.)
factors.add(/* ... */);
initialValues.insert(X(i), pose_initial_guess);
timestamps[X(i)] = current_time;

// Update ISAM2
auto result = smoother.update(factors, initialValues, timestamps);

// Get optimized result
Values optimized = smoother.calculateEstimate();
Pose3 pose = optimized.at<Pose3>(X(i));
```

### Extract Results
```cpp
// Get specific variable
Pose3 pose = optimized.at<Pose3>(X(i));
Vector3 velocity = optimized.at<Vector3>(V(i));
imuBias::ConstantBias bias = optimized.at<imuBias::ConstantBias>(B(i));

// Get covariance (optional)
Marginals marginals(smoother.getFactors(), optimized);
Matrix poseCov = marginals.marginalCovariance(X(i));
```

---

## Key Parameters

| Parameter | Values | Effect |
|-----------|--------|--------|
| `relinearizeThreshold` | 0.01-0.1 | Lower = more accurate but slower |
| `relinearizeSkip` | 1-10 | Lower = check more often |
| `smootherLag` | 1.0-5.0s | Larger = more context but slower |
| `factorization` | CHOLESKY/QR | CHOLESKY faster, QR more stable |

**Recommended Settings:**
```cpp
// Accurate (slower)
relinearizeThreshold = 0.01;
relinearizeSkip = 1;
smootherLag = 5.0;

// Fast (less accurate)
relinearizeThreshold = 0.1;
relinearizeSkip = 10;
smootherLag = 2.0;

// Balanced (recommended)
relinearizeThreshold = 0.05;
relinearizeSkip = 5;
smootherLag = 3.0;
```

---

---

## GP Motion Priors Quick Reference

### What are They?
Soft constraints that enforce smooth acceleration between states
- **Purpose**: Smooth trajectories, filter IMU noise
- **Cost**: ~30-50% slower optimization
- **Benefit**: More robust, physically realistic motion

### Quick Setup
```cpp
// Enable in GraphTimeCentricKimeraParams
kimeraParams_.addGPMotionPriors = true;
kimeraParams_.gpType = "WNOJ";  // "WNOJ", "Singer", or "WNOA"

// Configure Q_c (process noise)
// Format: [qx, qy, qz, qroll, qpitch, qyaw]
QcGPMotionPriorFull: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]  // Moderate
```

### Q_c Tuning
```cpp
// Very Smooth (reject IMU noise strongly)
Q_c = [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]

// Moderate (balanced)
Q_c = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]

// Dynamic (follow IMU closely)
Q_c = [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
```

**Rule of Thumb**: Increase Q_c by 10x if trajectory too smooth, decrease by 10x if too jerky

### Adding GP Priors Manually
```cpp
// In factor graph construction
auto gp_factor = boost::make_shared<fgo::factor::GPWNOJPrior>(
    X(i), V(i), W(i),      // State i: pose, velocity, angular velocity
    X(j), V(j), W(j),      // State j
    acc_i, acc_j,          // Reference accelerations (often zero)
    dt,                    // Time between states
    Qc_model,              // Noise model from Q_c
    false,                 // useAutoDiff
    true                   // calcJacobian
);
factors.add(gp_factor);
```

### When to Use
✅ **Use GP Priors When:**
- IMU is noisy
- States are sparse (> 0.1s apart)
- Need smooth trajectories
- Platform has smooth dynamics

❌ **Don't Use When:**
- Very high-quality IMU
- Very dense states (< 0.01s)
- Highly dynamic motion
- Tight computational budget

---

## Common Operations

### Check Optimization Quality
```cpp
ISAM2Result result = smoother.getISAM2Result();

// Did optimization work?
std::cout << "Error after: " << result.errorAfter.value() << std::endl;
// Should be decreasing or stable

// How many variables updated?
std::cout << "Variables relinearized: " 
          << result.variablesRelinearized << std::endl;
// Should be > 0 occasionally

// How many affected?
std::cout << "Cliques updated: " << result.cliques << std::endl;
```

### Monitor Performance
```cpp
auto start = std::chrono::high_resolution_clock::now();
smoother.update(factors, values, timestamps);
auto end = std::chrono::high_resolution_clock::now();

auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
std::cout << "Update: " << duration.count() << " ms" << std::endl;

// Typical times:
// - No relinearization: 5-20 ms
// - With relinearization: 50-200 ms
```

### Debug Information
```cpp
// Enable detailed results
params.enableDetailedResults = true;

// Print factor graph
smoother.getFactors().print("Current factors:");

// Print values
optimized.print("Optimized values:");

// Check specific variable exists
if (optimized.exists(X(i))) {
    std::cout << "State " << i << " exists" << std::endl;
}
```

---

## Factor Types in Your System

### CombinedImuFactor
```cpp
// Between consecutive states
auto imuFactor = CombinedImuFactor::create(
    X(i), V(i), X(i+1), V(i+1), B(i), B(i+1),
    preintegratedImu);
factors.add(imuFactor);

// What it does:
// - Integrates all IMU measurements between states
// - Provides relative pose/velocity constraint
// - Accounts for bias
```

### GPWNOJPrior (Gaussian Process Motion Prior)
```cpp
// Smooth motion constraint (White Noise On Jerk model)
auto gpPrior = boost::make_shared<fgo::factor::GPWNOJPrior>(
    X(i), V(i), W(i),     // State i: pose, velocity, angular velocity
    X(j), V(j), W(j),     // State j
    acc_i, acc_j,         // Reference accelerations
    delta_t,              // Time difference
    Qc_model);            // Process noise model
factors.add(gpPrior);

// What it does:
// - Enforces smooth acceleration (jerk is white noise)
// - Gaussian Process motion model
// - Prevents unrealistic motion (sudden changes)
// - Acts as soft constraint (not hard like IMU)
// - 6-way factor: connects 6 variable keys

// Alternatives:
// - GPSingerPrior: Exponentially decaying acceleration
// - GPWNOAPrior: White noise on acceleration (simpler)

// Variables involved:
// - X(i), X(j): Poses at times i and j
// - V(i), V(j): Velocities
// - W(i), W(j): Angular velocities (may be needed)
// - Does NOT use B (bias) - bias is IMU-specific
```

### Dual Constraints (IMU + GP)
```cpp
// Typical pattern: Both factors on same state pair
size_t i = state_idx;
size_t j = next_state_idx;

// 1. Add IMU factor (measurement constraint)
auto imuFactor = boost::make_shared<gtsam::CombinedImuFactor>(
    X(i), V(i), X(j), V(j), B(i), B(j), preintegrated_imu);
factors.add(imuFactor);

// 2. Add GP prior (smoothness constraint)
auto gpFactor = boost::make_shared<fgo::factor::GPWNOJPrior>(
    X(i), V(i), W(i), X(j), V(j), W(j), acc_i, acc_j, dt, Qc_model);
factors.add(gpFactor);

// Result: Joint optimization
// - IMU: "states should match integrated IMU data"
// - GP:  "acceleration should be smooth"
// - ISAM2 finds best balance between both constraints
```

### PriorFactor
```cpp
// Initial state constraint
auto prior = PriorFactor<Pose3>(
    X(0), initial_pose, prior_noise);
factors.add(prior);

// What it does:
// - Fixes global reference frame
// - Provides initial estimate
// - Prevents global drift
```

---

## Troubleshooting Guide

### Problem: Optimization Diverges
**Symptoms**: Error increases, estimates get worse

**Solutions**:
```cpp
// 1. Lower relinearization threshold
params.relinearizeThreshold = 0.01;
params.relinearizeSkip = 1;

// 2. Force relinearization
ISAM2UpdateParams updateParams;
updateParams.force_relinearize = true;
isam.update(factors, values, updateParams);

// 3. Check initial guesses
// Make sure they're reasonable, not NaN/Inf

// 4. Verify factor noise models
// Too small → overconfident → divergence
// Too large → under-constrained → drift
```

### Problem: Too Slow
**Symptoms**: Updates take >100ms consistently

**Solutions**:
```cpp
// 1. Reduce smoother window
smootherLag = 2.0;  // instead of 5.0

// 2. Relax relinearization
params.relinearizeThreshold = 0.1;
params.relinearizeSkip = 10;

// 3. Use Cholesky factorization
params.factorization = ISAM2Params::CHOLESKY;

// 4. Check number of active variables
size_t numVars = isam.getLinearizationPoint().size();
// Should be ~(smootherLag / dt) * 3
// If growing unbounded, marginalization isn't working
```

### Problem: Memory Leak
**Symptoms**: Memory grows over time

**Solutions**:
```cpp
// 1. Enable factor slot reuse
params.findUnusedFactorSlots = true;

// 2. Verify timestamps are set
timestamps[X(i)] = current_time;  // MUST set for each variable

// 3. Check marginalization
auto oldKeys = smoother.findKeysBefore(current_time - smootherLag);
std::cout << "Marginalizing " << oldKeys.size() << " keys" << std::endl;
// Should be > 0 regularly

// 4. Clear old buffers
// In your graph class, clear IMU buffers after use
```

### Problem: NaN/Inf Results
**Symptoms**: Results contain NaN or Inf

**Solutions**:
```cpp
// 1. Check initial values
assert(!initial_pose.translation().hasNaN());
assert(initial_pose.rotation().matrix().determinant() > 0);

// 2. Check factor noise models
// Make sure they're not zero or negative
auto noise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
assert((noise->sigmas().array() > 0).all());

// 3. Check IMU preintegration
assert(!preintegratedImu->deltaPij().hasNaN());
assert(!preintegratedImu->deltaVij().hasNaN());

// 4. Use QR factorization (more stable)
params.factorization = ISAM2Params::QR;
```

---

## Understanding the Output

### ISAM2Result Structure
```cpp
struct ISAM2Result {
    size_t variablesRelinearized;    // How many vars relinearized
    size_t variablesReeliminated;    // How many vars re-eliminated
    size_t cliques;                  // How many tree nodes updated
    boost::optional<double> errorBefore;  // Error before update
    boost::optional<double> errorAfter;   // Error after update
    FactorIndices newFactorsIndices;      // Indices of new factors
};

// Good signs:
// - errorAfter < errorBefore (or close)
// - variablesRelinearized > 0 occasionally
// - cliques is small (incremental update)

// Bad signs:
// - errorAfter >> errorBefore (diverging)
// - variablesRelinearized = 0 always (not relinearizing)
// - cliques = total variables (full re-solve)
```

### Interpreting Errors
```cpp
double error = result.errorAfter.value();

// What does error mean?
// Sum of squared residuals: Σ ||h(x) - z||²

// Typical values:
// < 1e3:  Excellent
// 1e3-1e6: Good
// 1e6-1e9: Acceptable
// > 1e9:   Problem (conflicting constraints or bad initialization)
```

---

## Quick Diagnostic Commands

```cpp
// Print current state
std::cout << "=== ISAM2 Status ===" << std::endl;
std::cout << "Variables: " << isam.getLinearizationPoint().size() << std::endl;
std::cout << "Factors: " << isam.getFactorsUnsafe().size() << std::endl;
std::cout << "Error: " << result.errorAfter.value() << std::endl;
std::cout << "Relinearized: " << result.variablesRelinearized << std::endl;

// Check specific variable
if (optimized.exists(X(i))) {
    Pose3 pose = optimized.at<Pose3>(X(i));
    std::cout << "X(" << i << "): " 
              << pose.translation().transpose() << std::endl;
} else {
    std::cout << "X(" << i << ") not found!" << std::endl;
}

// Verify marginalization
auto timestamps = smoother.timestamps();
double oldest = std::numeric_limits<double>::max();
for (const auto& kv : timestamps) {
    oldest = std::min(oldest, kv.second);
}
std::cout << "Oldest state timestamp: " << oldest << std::endl;
std::cout << "Should be > " << (current_time - smootherLag) << std::endl;
```

---

## Configuration File Template

```yaml
# ISAM2 Configuration
Optimizer:
  smootherType: "IncrementalFixedLag"
  smootherLag: 3.0  # seconds
  
  ISAM2:
    # Optimization method: "GN" or "DOGLEG"
    optimizationParams: "GN"
    
    # Relinearization settings
    relinearizeThreshold: 0.05  # 0.01-0.1
    relinearizeSkip: 5          # 1-10
    
    # Linear algebra
    factorization: "CHOLESKY"   # "CHOLESKY" or "QR"
    
    # Debug/optimization
    enableDetailedResults: false
    findUnusedFactorSlots: true  # For fixed-lag
    
  # GP Motion Prior (if enabled)
  gpType: "WNOJ"
  QcGPWNOJMotionPriorFull: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  
  # IMU settings
  gyro_noise_sigma: 0.001
  accel_noise_sigma: 0.01
  gravity: [0, 0, -9.81]
```

---

## Key Concepts in 5 Seconds

- **Factor Graph**: States (variables) + Measurements (factors) = Optimization problem
- **ISAM2**: Incremental solver - only update what changed
- **Bayes Tree**: Internal data structure for efficiency
- **Relinearization**: Re-compute linear approximation when drift is large
- **Marginalization**: Remove old states, keep their information as prior
- **Fixed-Lag**: Keep only recent states, marginalize the rest

---

## Resources

- **Full Tutorial**: `ISAM2_TUTORIAL.md`
- **Visual Examples**: `ISAM2_VISUAL_REFERENCE.md`
- **Kimera Integration**: `KIMERA_INTEGRATION_QUICK_REFERENCE.md`
- **GTSAM Docs**: https://gtsam.org/
- **Paper**: "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree"

---

## When to Use ISAM2 vs Batch

**Use ISAM2 when:**
✅ Data arrives incrementally (streaming)
✅ Need real-time performance
✅ Problem is sparse (sequential structure)
✅ Want constant-time updates (fixed-lag)

**Use Batch (LM/GN) when:**
❌ Have all data upfront
❌ Offline processing OK
❌ Small problem (< 100 variables)
❌ Need absolute best accuracy (but slower)

**Your Case**: Kimera streams data → ISAM2 is perfect! 🎯
