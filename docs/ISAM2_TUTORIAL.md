# ISAM2 Tutorial: Understanding Incremental Smoothing and Mapping

## Table of Contents
1. [What is ISAM2?](#what-is-isam2)
2. [Core Concepts](#core-concepts)
3. [How ISAM2 Works](#how-isam2-works)
4. [ISAM2 in Your Codebase](#isam2-in-your-codebase)
5. [Gaussian Process Motion Priors](#gaussian-process-motion-priors)
6. [Kimera Integration Context](#kimera-integration-context)
7. [Configuration Parameters](#configuration-parameters)
8. [Practical Usage Patterns](#practical-usage-patterns)
9. [Performance Considerations](#performance-considerations)

---

## What is ISAM2?

**ISAM2 (Incremental Smoothing and Mapping 2)** is a highly efficient algorithm for **incremental nonlinear optimization** of factor graphs. It's the backbone of most modern SLAM and sensor fusion systems.

### Key Characteristics

- **Incremental**: Updates the solution as new data arrives, rather than re-solving from scratch
- **Efficient**: Only relinearizes and updates parts of the graph that significantly changed
- **Exact**: Maintains the same solution quality as batch optimization
- **Adaptive**: Automatically manages variable ordering and linearization points

### What Problem Does It Solve?

In robotics/navigation, you have:
- **States**: Robot poses (X), velocities (V), IMU biases (B), landmarks, etc.
- **Measurements**: IMU data, GPS, visual features, etc.
- **Constraints**: How measurements relate states (e.g., "IMU data connects two poses")

ISAM2 finds the **Maximum A Posteriori (MAP)** estimate of all states given all measurements by solving:

```
θ* = argmin_θ Σ ||h_i(θ) - z_i||²_Σi
```

Where:
- `θ` = all state variables
- `h_i(θ)` = prediction function for measurement i
- `z_i` = actual measurement i
- `Σi` = measurement covariance

---

## Core Concepts

### 1. Factor Graph Representation

Your optimization problem is represented as a **Bipartite Graph**:

```
States (Variables):          X₀    X₁    X₂    X₃
                              |     |     |     |
Factors (Constraints):    [Prior]-[IMU]-[IMU]-[GPS]
                                   |     |
                                 [IMU]   |
                                         |
                                      [Prior]
```

**Nodes (Variables)**: Things you want to estimate
- `X(i)`: Pose at time i
- `V(i)`: Velocity at time i  
- `B(i)`: IMU bias at time i

**Edges (Factors)**: Constraints/measurements
- `PriorFactor`: Initial guess with uncertainty
- `CombinedImuFactor`: Integrates IMU between two poses
- `GPMotorPrior`: Gaussian Process motion model
- `BetweenFactor`: Relative constraints

### 2. Bayes Tree

ISAM2 internally maintains a **Bayes Tree** - a data structure that:

```
                    Root
                   /    \
              X₀,X₁      X₃
                |         |
               V₀        V₃,B₃
                |
               B₀
```

- Encodes the factored joint probability distribution
- Enables incremental updates without full re-solve
- Supports efficient marginalization of old variables
- Maintains sparsity for fast computation

### 3. Linearization

Nonlinear factors are linearized around the current estimate:

```
Original nonlinear:  f(x) ≈ f(x₀) + J(x₀)·Δx

Where:
  x₀ = current linearization point
  J = Jacobian matrix
  Δx = delta from linearization point
```

ISAM2 tracks when linearization points become stale and **relinearizes** as needed.

---

## How ISAM2 Works

### The Update Cycle

```
┌─────────────────────────────────────────────────┐
│ 1. NEW DATA ARRIVES                             │
│    - New factors (measurements)                 │
│    - New variables (new states)                 │
│    - Optional: factors to remove                │
└────────────────┬────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│ 2. ADD TO FACTOR GRAPH                          │
│    - Append new factors to internal graph       │
│    - Initialize new variables with prior guess  │
└────────────────┬────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│ 3. CHECK RELINEARIZATION                        │
│    - Every N updates (relinearizeSkip)          │
│    - When delta exceeds threshold               │
│    - Force if requested                         │
└────────────────┬────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│ 4. INCREMENTAL UPDATE                           │
│    - Identify affected variables ("wildfire")   │
│    - Update only affected subtree               │
│    - Maintain Bayes Tree structure              │
└────────────────┬────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│ 5. CALCULATE ESTIMATE                           │
│    - Current best estimate: θ = θ₀ + Δθ        │
│    - Extract desired variables                  │
│    - Compute marginal covariances if needed     │
└─────────────────────────────────────────────────┘
```

### Key Algorithms

#### Wildfire Algorithm
When a variable's linearization point changes significantly:
1. Mark it for relinearization
2. Propagate relinearization to connected variables ("wildfire")
3. Update only affected cliques in Bayes Tree

#### Variable Ordering
ISAM2 automatically determines good variable elimination ordering:
- Variables eliminated first become "deeper" in tree
- Later variables stay near root (easy to update)
- In fixed-lag smoothing: old variables are marginalized out

---

## ISAM2 in Your Codebase

### Class Hierarchy

```
gtsam::ISAM2 (base solver)
      ↑
      │ wraps
      │
fgo::solvers::IncrementalFixedLagSmoother
      ↑
      │ uses
      │
fgo::graph::GraphTimeCentric[Kimera]
      ↑
      │ accessed via
      │
fgo::integration::KimeraIntegrationInterface
      ↑
      │ called by
      │
Kimera-VIO::GraphTimeCentricBackendAdapter
```

### Your IncrementalFixedLagSmoother

Located: `online_fgo_core/src/solver/IncrementalFixedLagSmoother.cpp`

**Key features:**
```cpp
class IncrementalFixedLagSmoother {
    gtsam::ISAM2 isam_;  // The core ISAM2 solver
    
    // Updates with new factors and manages sliding window
    Result update(
        const NonlinearFactorGraph& newFactors,
        const Values& newTheta,
        const KeyTimestampMap& timestamps);
};
```

**What it does:**
1. **Maintains sliding window**: Keeps only recent states (smootherLag seconds)
2. **Marginalizes old states**: Removes variables outside the window
3. **Constrains ordering**: Ensures marginalizable variables are eliminated first
4. **Handles timestamps**: Maps state keys to real timestamps

### Typical Update Flow in Your Code

```cpp
// In GraphTimeCentric::optimize()

// 1. Build factor graph from buffered data
NonlinearFactorGraph graph;
Values initialValues;

// Add IMU factors between consecutive states
for (each state pair) {
    auto imuFactor = CombinedImuFactor::create(...);
    graph.add(imuFactor);
}

// Add GP motion priors if enabled
if (useGPMotionPrior) {
    graph.add(GPWNOJPrior6(...));
}

// 2. Call smoother update
auto result = incrementalSmootherPtr_->update(
    graph,           // New factors
    initialValues,   // Initial guesses for new variables
    timestamps,      // Map: Key -> timestamp
    factorsToRemove  // Optional: old factors to remove
);

// 3. Extract optimized values
Values optimizedValues = incrementalSmootherPtr_->calculateEstimate();

// 4. Update internal state
currentStateData_.state = optimizedValues.at<Pose3>(X(currentIndex));
currentStateData_.velocity = optimizedValues.at<Vector3>(V(currentIndex));
currentStateData_.imuBias = optimizedValues.at<imuBias::ConstantBias>(B(currentIndex));
```

---

## Kimera Integration Context

### How Kimera Uses Your ISAM2 Backend

```
Kimera VioBackend (per keyframe)
        │
        │ buffers IMU data
        │
        ▼
   [IMU₁, IMU₂, ..., IMUₙ]
        │
        │ on keyframe/optimization trigger
        │
        ▼
GraphTimeCentricBackendAdapter::optimizeGraph()
        │
        │ converts to FGO format
        │
        ▼
KimeraIntegrationInterface::optimize()
        │
        │ constructs factor graph
        │
        ▼
GraphTimeCentricKimera::optimizeWithExternalFactors()
        │
        │ builds graph incrementally
        │
        ▼
IncrementalFixedLagSmoother::update()
        │
        │ calls ISAM2
        │
        ▼
gtsam::ISAM2::update(graph, values)
        │
        │ incremental optimization
        │
        ▼
[Optimized X, V, B states]
```

### What ISAM2 Sees From Kimera

**Variables Created:**
- `X(i)`: Pose at keyframe i (from Kimera's timestamp)
- `V(i)`: Velocity at keyframe i
- `B(i)`: IMU bias at keyframe i

**Factors Added:**
- `CombinedImuFactor`: Between consecutive keyframes
  - Integrates all IMU measurements between timestamps
  - Includes preintegrated measurements and Jacobians
  - Covariance from IMU noise model

- `GPMotorPrior`: Between consecutive states (if enabled)
  - Gaussian Process motion model
  - Enforces smooth motion
  - Configurable Q_c (process noise)

- `PriorFactor`: On first state
  - Initial pose/velocity/bias estimate from Kimera
  - Sets the global reference frame

**Example factor graph after 3 keyframes:**
```
States:  X(0)    V(0)    B(0)    X(1)    V(1)    B(1)    X(2)    V(2)    B(2)
          |       |       |       |       |       |       |       |       |
Factors: [------- PriorFactor -------]
                  |       |       |
          [------ CombinedImuFactor ------]
                              |       |       |
                      [------- GPPrior -------]
                                      |       |       |
                              [------ CombinedImuFactor ------]
                                                      |       |       |
                                              [------- GPPrior -------]
```

---

## Gaussian Process Motion Priors

### What are GP Motion Priors?

**Gaussian Process (GP) Motion Priors** are soft constraints that enforce smooth, physically plausible motion between consecutive states. Unlike hard constraints (like IMU factors that must be satisfied), GP priors add a **preference** for smooth trajectories.

**The Core Idea:**
```
Without GP Priors:
  State transitions limited only by IMU measurements
  Can have jerky, non-smooth trajectories
  Vulnerable to IMU noise and drift

With GP Priors:
  State transitions must also satisfy smoothness constraints
  Enforces continuous acceleration profiles
  More robust to IMU anomalies
  Better trajectory quality
```

### Why Add GP Priors to a Time-Centric Factor Graph?

Your time-centric system with IMU factors already provides odometry constraints. GP priors **complement** these by:

1. **Smoothing Acceleration**: Penalizes sudden changes in acceleration
2. **Handling Sparse States**: When states are far apart in time, GP priors interpolate smoothly
3. **Regularization**: Prevents overfitting to noisy IMU data
4. **Physical Realism**: Enforces dynamics that real robots/vehicles follow

**Trade-off:**
- **Pro**: Smoother trajectories, better noise rejection
- **Con**: Additional computational cost per state pair
- **Pro**: Can reduce required IMU integration frequency
- **Con**: Adds tuning parameters (Q_c matrix)

---

### GP Prior Theory

#### Mathematical Foundation

A Gaussian Process motion prior models the continuous-time trajectory as a stochastic process with:

**State Representation:**
```
ξ(t) = [p(t); R(t)]      // Pose (position + rotation)
ν(t) = [v(t); ω(t)]      // Velocity (linear + angular)
α(t) = [a(t); α_rot(t)]  // Acceleration (linear + rotational)
```

**GP Assumption:**
```
α(t) ~ GP(0, Q_c · δ(t - t'))
```
Where:
- `Q_c`: Power spectral density (PSD) matrix - tunable process noise
- `δ(·)`: Dirac delta function (white noise in continuous time)

**This means**: Acceleration changes are modeled as white noise with covariance `Q_c`.

#### Integration Over Time

Given two states at times `t_i` and `t_j` with `Δt = t_j - t_i`:

**State Transition Model:**
```
ξ_j = ξ_i ⊕ (ν_i · Δt) ⊕ (½ · α_i · Δt²)
ν_j = ν_i + α_i · Δt
```

Where `⊕` is manifold addition (accounting for rotation manifold).

**The GP Prior Factor:**
Constrains the states to follow this transition model with uncertainty defined by `Q_c` integrated over `Δt`.

**Error Function:**
```
e = Φ_inv(Δξ, Δν, α_i, α_j, Δt)

Where Φ_inv is the inverse GP motion model
```

The prior adds a cost:
```
cost = ||e||²_Σ

Where Σ = Q(Δt) = ∫∫ transition_jacobian · Q_c · transition_jacobian^T dt
```

---

### GP Prior Types in Your System

Your codebase implements several GP prior variants:

#### 1. GPWNOJ (White Noise On Jerk) - **Default**

**File**: `include/online_fgo_core/factor/motion/GPWNOJPrior.h`

**Assumption**: Jerk (derivative of acceleration) is white noise

**Variables Connected:**
- `Pose_i`, `Velocity_i`, `Omega_i` (angular velocity)
- `Pose_j`, `Velocity_j`, `Omega_j`

**Total**: 6 variable keys (6-way factor)

**Model:**
```
d³p/dt³ ~ N(0, Q_c)    // White noise on jerk
```

**Use Case**: General-purpose motion modeling
- **Best for**: Vehicles, robots with relatively smooth motion
- **Computational**: Medium complexity
- **Tuning**: Single Q_c matrix (6×6)

#### 2. GPSinger

**File**: `include/online_fgo_core/factor/motion/GPSingerPrior.h`

**Assumption**: Acceleration decays exponentially (Singer model)

**Model:**
```
dα/dt = -β · α + w(t)    // β is decay constant

Models maneuvers as exponentially decaying accelerations
```

**Use Case**: Vehicles with maneuvering behavior
- **Best for**: Cars, UAVs that accelerate then coast
- **Computational**: Slightly more complex than WNOJ
- **Tuning**: Q_c matrix + β (decay parameter)

#### 3. GPWNOA (White Noise On Acceleration)

**File**: `include/online_fgo_core/factor/motion/GPWNOAPrior.h`

**Assumption**: Acceleration itself is white noise (simpler model)

**Model:**
```
d²p/dt² ~ N(0, Q_c)    // White noise on acceleration
```

**Use Case**: Very smooth, constant-velocity-ish motion
- **Best for**: Slow-moving robots, indoor navigation
- **Computational**: Lowest complexity
- **Tuning**: Single Q_c matrix (6×6)

---

### How GP Priors Integrate with ISAM2

#### Factor Graph Structure

**Without GP Priors:**
```
States:  X(0)─────X(1)─────X(2)─────X(3)
         │        │        │        │
         V(0)     V(1)     V(2)     V(3)
         │        │        │        │
         B(0)     B(1)     B(2)     B(3)

Factors: [Prior]
              │
         [IMU Factor]────────X(1),V(1),B(1)
                              │
                         [IMU Factor]────────X(2),V(2),B(2)
                                              │
                                         [IMU Factor]────────X(3),V(3),B(3)

Factor count: 1 prior + 3 IMU factors = 4 total
```

**With GP Priors Added:**
```
States:  X(0)─────X(1)─────X(2)─────X(3)
         │        │        │        │
         V(0)     V(1)     V(2)     V(3)
         │        │        │        │
         Ω(0)     Ω(1)     Ω(2)     Ω(3)     ← May need angular velocity
         │        │        │        │
         B(0)     B(1)     B(2)     B(3)

Factors: [Prior]
              │
         [IMU Factor]────────X(1),V(1),B(1)
              │               │
         [GP Prior ]──────────┘
              │
         [IMU Factor]────────X(2),V(2),B(2)
              │               │
         [GP Prior ]──────────┘
              │
         [IMU Factor]────────X(3),V(3),B(3)
              │               │
         [GP Prior ]──────────┘

Factor count: 1 prior + 3 IMU + 3 GP = 7 total
```

**Key Observations:**
1. GP priors connect **same variable pairs** as IMU factors
2. GP priors add **additional constraints** (not replacements)
3. Each state pair has **two factors**: IMU (measurement) + GP (smoothness)
4. Total factors grows linearly: `N_factors = 1 + 2(N_states - 1)`

---

### ISAM2 Optimization with GP Priors

#### How ISAM2 Handles Dual Constraints

When both IMU factor and GP prior connect the same states:

**Information Fusion:**
```
Total Information = IMU Information + GP Information

In terms of cost:
J = ||e_imu||²_Σ_imu + ||e_gp||²_Σ_gp

ISAM2 jointly optimizes both simultaneously
```

**Effect on Optimization:**

1. **Tighter Constraints**:
   ```
   States more tightly coupled
   → More entries in Bayes Tree
   → Slightly more variables affected during updates
   ```

2. **Information Matrix**:
   ```
   Without GP:     With GP:
   [● · · ·]       [● ● · ·]
   [· ● · ·]       [● ● ● ·]
   [· · ● ·]       [· ● ● ●]
   [· · · ●]       [· · ● ●]
   
   More fill-in = denser = slower (but better conditioned)
   ```

3. **Relinearization Impact**:
   ```
   GP priors depend on current linearization point
   → May trigger more frequent relinearization
   → Especially important when acceleration changes
   ```

#### Performance Impact

**Computational Cost Per Update:**
```
Without GP:                    With GP:
─────────────────────────────────────────────────
Factor creation:    10 ms     Factor creation:    15 ms (+50%)
ISAM2 update:       20 ms     ISAM2 update:       30 ms (+50%)
Total:              30 ms     Total:              45 ms (+50%)

Typical overhead: 30-50% increase in optimization time
```

**But with benefits:**
- Fewer relinearizations may be needed (smoother trajectory)
- Can use fewer states for same accuracy
- Better convergence properties

**Memory:**
```
Each GP prior factor adds:
- Factor object: ~500 bytes
- Covariance matrix: ~300 bytes (6×6 doubles)
- Jacobian storage: ~600 bytes
Total per GP prior: ~1.5 KB

For 100 state pairs: ~150 KB additional
```

---

### Configuration and Tuning

#### Q_c Matrix - The Core Parameter

The `Q_c` matrix defines the **power spectral density** of the acceleration noise.

**Structure:**
```cpp
Q_c = diag([qx, qy, qz, qroll, qpitch, qyaw])

Where:
  qx, qy, qz: Linear acceleration noise (m/s³)²
  qroll, qpitch, qyaw: Rotational acceleration noise (rad/s³)²
```

**Physical Interpretation:**
- **Larger Q_c**: More allowed acceleration variation
  - Looser smoothness constraint
  - Follows IMU more closely
  - Good for: Dynamic motion, high maneuverability
  
- **Smaller Q_c**: Less allowed acceleration variation
  - Tighter smoothness constraint
  - Smoother trajectory
  - Good for: Smooth platforms, rejecting IMU noise

**Tuning Guidelines:**

```cpp
// Very Smooth (Indoor robot, slow motion)
Q_c = diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])

// Moderate (Typical vehicle)
Q_c = diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])

// Dynamic (Aggressive UAV, sports car)
Q_c = diag([100.0, 100.0, 100.0, 10.0, 10.0, 10.0])

// Very Dynamic (Essentially disable GP prior)
Q_c = diag([1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0])
```

#### Configuration in GraphBase.cpp

```cpp
// Load GP type
auto gpTypeStr = appPtr_->getParameters().getString(
    "GNSSFGO.Optimizer.gpType", "WNOJ");

// Load Q_c for motion priors
auto QcGPMotionPriorFullVec = appPtr_->getParameters().getDoubleArray(
    "GNSSFGO.Optimizer.QcGPWNOJMotionPriorFull", 
    std::vector<double>(6, 1000.0));  // Default: very loose

graphBaseParamPtr_->QcGPMotionPriorFull = 
    gtsam::Vector6(QcGPMotionPriorFullVec.data());

// Load Q_c for interpolators (if using GP interpolation)
auto QcGPInterpolatorFullVec = appPtr_->getParameters().getDoubleArray(
    "GNSSFGO.Optimizer.QcGPWNOJInterpolatorFull",
    std::vector<double>(6, 1000.0));

graphBaseParamPtr_->QcGPInterpolatorFull = 
    gtsam::Vector6(QcGPInterpolatorFullVec.data());
```

#### YAML Configuration Example

```yaml
GNSSFGO:
  Optimizer:
    # GP Type: "WNOJ", "Singer", "WNOA"
    gpType: "WNOJ"
    
    # Q_c for GP Motion Priors (between states)
    # Order: [qx, qy, qz, qroll, qpitch, qyaw]
    QcGPWNOJMotionPriorFull: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
    
    # Q_c for GP Interpolators (for measurements between states)
    QcGPWNOJInterpolatorFull: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
    
    # Optional: Singer model specific
    # SingerDecayConstant: 0.1
```

---

### Implementation in GraphTimeCentricKimera

#### Adding GP Priors During Factor Graph Construction

**Code Flow:**
```cpp
// In GraphTimeCentricKimera::constructFactorGraphFromTimestamps()

// 1. Create states at timestamps
std::vector<size_t> created_states;
for (const auto& ts : timestamps) {
  size_t state_idx = findOrCreateStateForTimestamp(ts, true);
  created_states.push_back(state_idx);
}

// 2. Add IMU factors between consecutive states
for (size_t i = 1; i < created_states.size(); ++i) {
  addIMUFactorBetweenStates(created_states[i-1], created_states[i], imu_data);
}

// 3. Add GP motion priors (if enabled)
if (kimeraParams_.addGPMotionPriors) {
  addGPMotionPriorsForStates(created_states);  // ← This is new
}
```

#### GP Prior Creation Details

```cpp
bool GraphTimeCentricKimera::addGPMotionPriorBetweenStates(
    size_t state_i_idx, size_t state_j_idx) {
  
  // Get GTSAM keys
  gtsam::Key pose_i = X(state_i_idx);
  gtsam::Key vel_i = V(state_i_idx);
  gtsam::Key omega_i = W(state_i_idx);  // Angular velocity (if needed)
  gtsam::Key pose_j = X(state_j_idx);
  gtsam::Key vel_j = V(state_j_idx);
  gtsam::Key omega_j = W(state_j_idx);
  
  // Get time delta
  double dt = keyTimestampMap_[pose_j] - keyTimestampMap_[pose_i];
  
  // Get Q_c matrix from parameters
  gtsam::SharedNoiseModel Qc_model = 
      gtsam::noiseModel::Diagonal::Variances(
          graphBaseParamPtr_->QcGPMotionPriorFull);
  
  // Estimated accelerations (often zero or from previous state)
  gtsam::Vector6 acc_i = gtsam::Vector6::Zero();
  gtsam::Vector6 acc_j = gtsam::Vector6::Zero();
  
  // Create GP prior factor (example: WNOJ)
  auto gp_factor = boost::make_shared<fgo::factor::GPWNOJPrior>(
      pose_i, vel_i, omega_i,
      pose_j, vel_j, omega_j,
      acc_i, acc_j,
      dt,
      Qc_model,
      false,  // useAutoDiff
      true    // calcJacobian
  );
  
  // Add to graph
  this->push_back(gp_factor);
  
  return true;
}
```

**Key Points:**
1. GP prior uses **same variable keys** as IMU factor
2. `dt` must match the time between states
3. `acc_i` and `acc_j` are reference accelerations (often zero)
4. Noise model computed from `Q_c` integrated over `dt`

---

### What Changes in the Time-Centric System

#### Before GP Priors (IMU-Only)

```
Kimera Keyframe at t=1.0s
        ↓
Create State X(1), V(1), B(1)
        ↓
Extract IMU data from t=0.8s to t=1.0s
        ↓
Preintegrate IMU → CombinedImuFactor
        ↓
Add factor: IMU connects X(0),V(0),B(0) ↔ X(1),V(1),B(1)
        ↓
ISAM2 Update
        ↓
Result: State estimate constrained by IMU measurements only
```

**Characteristics:**
- ✅ Direct measurement constraint (hard constraint)
- ✅ Accounts for IMU noise via covariance
- ❌ No smoothness enforcement
- ❌ Vulnerable to IMU spikes/anomalies
- ❌ Can produce jerky trajectories

#### After Adding GP Priors

```
Kimera Keyframe at t=1.0s
        ↓
Create State X(1), V(1), B(1), [W(1) if using full model]
        ↓
Extract IMU data from t=0.8s to t=1.0s
        ↓
Preintegrate IMU → CombinedImuFactor
        ↓
Add IMU factor: connects X(0),V(0),B(0) ↔ X(1),V(1),B(1)
        ↓
Compute Δt = t(1) - t(0) = 0.2s
        ↓
Add GP Prior: connects X(0),V(0),W(0) ↔ X(1),V(1),W(1)
        ↓
ISAM2 Update (now with both constraints)
        ↓
Result: State estimate is:
  1. Consistent with IMU measurements (IMU factor)
  2. Smoothly varying in acceleration (GP prior)
  3. Trade-off governed by respective noise models
```

**Characteristics:**
- ✅ Measurement constraint (IMU factor)
- ✅ Smoothness constraint (GP prior)
- ✅ Robust to IMU anomalies (GP smooths them out)
- ✅ More physically realistic trajectories
- ⚠️  Slightly slower optimization (~30-50% overhead)
- ⚠️  Requires tuning Q_c parameter

---

### Optimization Behavior Changes

#### Cost Function

**Without GP:**
```
J = ||e_imu(X,V,B)||²_Σ_imu + ||e_prior(X₀,V₀,B₀)||²_Σ_prior

Minimizes: IMU prediction errors only
```

**With GP:**
```
J = ||e_imu(X,V,B)||²_Σ_imu 
  + ||e_gp(X,V,W)||²_Σ_gp
  + ||e_prior(X₀,V₀,B₀)||²_Σ_prior

Minimizes: IMU errors + acceleration smoothness violations
```

#### Solution Behavior

**Scenario: IMU measurement spike (e.g., sensor glitch)**

Without GP Priors:
```
t=0: X₀ = [0, 0, 0]
t=1: X₁ = [1, 0, 0]    ← normal
t=2: X₂ = [5, 2, 0]    ← large jump (IMU spike)
t=3: X₃ = [6, 2, 0]    ← back to normal

Trajectory follows measurements blindly
```

With GP Priors (Q_c = moderate):
```
t=0: X₀ = [0, 0, 0]
t=1: X₁ = [1, 0, 0]      ← normal
t=2: X₂ = [2.5, 0.5, 0]  ← smoothed (GP resists large jump)
t=3: X₃ = [4, 1, 0]      ← smoother recovery

Trajectory balances IMU data with smoothness
```

**Effect:**
- GP prior acts as **low-pass filter** on acceleration
- Reduces impact of high-frequency IMU noise
- Trade-off controlled by `Q_c` (smaller = stronger filtering)

---

### Practical Guidelines

#### When to Use GP Priors

**✅ Use GP Priors When:**
1. IMU has significant noise/bias
2. States are sparse (> 0.1s apart)
3. Platform has smooth dynamics (vehicles, indoor robots)
4. You need physically realistic trajectories
5. Kimera visual-inertial has drift issues

**❌ Don't Use GP Priors When:**
1. IMU is very high quality (e.g., tactical grade)
2. States are very dense (< 0.01s apart, redundant)
3. Platform is highly dynamic (race car, acrobatic UAV)
4. Computational budget is very tight
5. You need to follow IMU exactly (e.g., calibration)

#### Integration with Kimera

In your Kimera integration:

```cpp
// GraphTimeCentricKimeraParams configuration
struct GraphTimeCentricKimeraParams {
  // ... other params ...
  
  // GP Motion Prior Control
  bool addGPMotionPriors = true;           // Enable/disable
  std::string gpType = "WNOJ";             // "WNOJ", "Singer", "WNOA"
  gtsam::Vector6 QcGPMotionPrior = 
      (gtsam::Vector6() << 10, 10, 10, 1, 1, 1).finished();
  
  // Optional: Different Q_c for different motion phases
  gtsam::Vector6 QcGPMotionPriorDynamic = 
      (gtsam::Vector6() << 100, 100, 100, 10, 10, 10).finished();
  bool useAdaptiveQc = false;              // Switch Q_c based on motion
};
```

**Adaptive Strategy:**
```cpp
// In addGPMotionPriorBetweenStates():
gtsam::Vector6 Qc;
if (detectingHighDynamics(state_i, state_j)) {
  Qc = kimeraParams_.QcGPMotionPriorDynamic;  // Loose
} else {
  Qc = kimeraParams_.QcGPMotionPrior;         // Tight
}
```

#### Debugging GP Prior Issues

**Problem: Optimization diverges after adding GP priors**
- **Cause**: Q_c too small, over-constraining system
- **Solution**: Increase Q_c by 10x, gradually reduce

**Problem: Trajectory still jerky with GP priors**
- **Cause**: Q_c too large, GP prior has no effect
- **Solution**: Decrease Q_c by 10x, check noise model

**Problem: Very slow optimization**
- **Cause**: GP priors creating dense factor graph
- **Solution**: 
  - Increase `relinearizeSkip`
  - Use sparser states
  - Consider disabling for high-rate states

**Problem: GP and IMU factors conflict**
- **Symptoms**: High optimization error, poor convergence
- **Cause**: IMU noise model and Q_c inconsistent
- **Solution**: Ensure `Σ_imu` and `Σ_gp` are compatible scales

---

## Configuration Parameters

// 1. Optimization Algorithm
isam2Params.optimizationParams = 
    gtsam::ISAM2GaussNewtonParams();  // Default: Gauss-Newton
    // OR
    gtsam::ISAM2DoglegParams();       // Alternative: Dogleg (trust region)

// 2. Relinearization Threshold
isam2Params.relinearizeThreshold = 0.1;  // Default: 0.1
```
**What it means**: Relinearize variables when their delta exceeds this value.
- Lower = more accurate but slower (more relinearization)
- Higher = faster but may diverge if too high
- **Recommendation**: 0.01-0.1 for accurate results

```cpp
// 3. Relinearization Skip
isam2Params.relinearizeSkip = 10;  // Default: 10
```
**What it means**: Only check for relinearization every N updates.
- Lower = more frequent checks (more accurate, slower)
- Higher = fewer checks (faster, less accurate)
- **Recommendation**: 1-10 depending on dynamics

```cpp
// 4. Factorization Method
isam2Params.factorization = gtsam::ISAM2Params::CHOLESKY;  // Default
// OR
isam2Params.factorization = gtsam::ISAM2Params::QR;
```
**What it means**: Linear algebra method for solving systems.
- CHOLESKY: Faster, requires positive definite systems
- QR: More stable, works with rank-deficient systems
- **Recommendation**: CHOLESKY for well-constrained problems

```cpp
// 5. Detailed Results
isam2Params.enableDetailedResults = false;  // Default: false
```
**What it means**: Return detailed statistics about the update.
- `true`: Get clique statistics, relinearization info
- Useful for debugging, adds overhead
- **Recommendation**: Enable only during development

```cpp
// 6. Find Unused Factor Slots
isam2Params.findUnusedFactorSlots = false;  // Default: false
```
**What it means**: Reuse slots when removing factors.
- `true`: More memory efficient when removing many factors
- Adds overhead to search for slots
- **Recommendation**: `true` for fixed-lag smoothing

### IncrementalFixedLagSmoother Parameters

```cpp
// Smoother lag (seconds)
double smootherLag = 3.0;  // Keep last 3 seconds of data

IncrementalFixedLagSmoother smoother(smootherLag, isam2Params);
```

**What smootherLag controls:**
- How many seconds of states to keep in the optimization
- Older states are marginalized out
- Trade-off: larger window = more accurate but slower

**Example**:
- `smootherLag = 1.0`: Very fast, only recent context
- `smootherLag = 5.0`: Slower, more global consistency
- `smootherLag = ∞`: Batch optimization (full SLAM)

---

## Practical Usage Patterns

### Pattern 1: Incremental Addition (Kimera's Pattern)

```cpp
// Initialize
ISAM2Params params;
params.relinearizeThreshold = 0.01;
params.relinearizeSkip = 1;
ISAM2 isam(params);

// For each new measurement
NonlinearFactorGraph newFactors;
Values newValues;

// Add factors for current observation
newFactors.add(/* your factors */);
newValues.insert(/* new variables */);

// Update incrementally
ISAM2Result result = isam.update(newFactors, newValues);

// Get current estimate
Values currentEstimate = isam.calculateEstimate();
```

### Pattern 2: Fixed-Lag Smoothing (Your Pattern)

```cpp
// Setup with time window
IncrementalFixedLagSmoother smoother(smootherLag, params);

// Each update
KeyTimestampMap timestamps;
timestamps[X(i)] = current_time;
timestamps[V(i)] = current_time;
timestamps[B(i)] = current_time;

auto result = smoother.update(
    newFactors, 
    newValues, 
    timestamps
);

// Automatically marginalizes states older than:
// current_time - smootherLag
```

### Pattern 3: Batch Then Incremental

```cpp
// 1. Initial batch solve
NonlinearFactorGraph batchGraph;
Values batchInitial;
// ... add all initial factors and values ...

LevenbergMarquardtOptimizer batchOptimizer(batchGraph, batchInitial);
Values batchResult = batchOptimizer.optimize();

// 2. Initialize ISAM2 with batch result
ISAM2 isam(params);
isam.update(batchGraph, batchResult);

// 3. Now do incremental updates
// ... as new data arrives ...
```

### Pattern 4: Constrained Variable Ordering

```cpp
// Force certain variables to be eliminated first
FastMap<Key, int> constrainedKeys;

// Group 0: marginalize these first
constrainedKeys[X(0)] = 0;
constrainedKeys[V(0)] = 0;
constrainedKeys[B(0)] = 0;

// Group 1: keep these longer
constrainedKeys[X(1)] = 1;
// ... etc ...

ISAM2UpdateParams updateParams;
updateParams.constrainedKeys = constrainedKeys;

isam.update(newFactors, newValues, updateParams);
```

---

## Performance Considerations

### Computational Complexity

**Without ISAM2 (Batch):**
- Time: O(n³) where n = total number of variables
- Memory: O(n²)
- **Example**: 1000 states → ~1 billion operations

**With ISAM2 (Incremental):**
- Time: O(m·k²) where m = new variables, k = affected variables
- Memory: O(n·k)
- **Example**: Adding 10 states affecting 20 → ~8,000 operations

**Speedup**: Often 100-1000x faster than batch!

### What Makes ISAM2 Fast?

1. **Incremental updates**: Only recompute what changed
2. **Sparse matrices**: Exploits problem structure
3. **Lazy relinearization**: Only when necessary
4. **Efficient ordering**: Keeps recent variables "active"

### Performance Tips

#### 1. Tune Relinearization
```cpp
// More accuracy (slower)
params.relinearizeThreshold = 0.01;
params.relinearizeSkip = 1;

// More speed (less accurate)
params.relinearizeThreshold = 0.1;
params.relinearizeSkip = 10;
```

#### 2. Use Fixed-Lag Smoothing
```cpp
// Don't let the problem grow unbounded
// Marginalize old states to maintain constant complexity
smootherLag = 3.0;  // seconds
```

#### 3. Smart Factor Removal
```cpp
// When removing factors, track their indices
FactorIndices toRemove;
toRemove.push_back(oldFactorIndex);

isam.update(newFactors, newValues, toRemove);
```

#### 4. Batch Initialization
```cpp
// For first N measurements, do batch solve
if (numUpdates < 10) {
    // Batch optimization
} else {
    // Switch to ISAM2 incremental
}
```

### Memory Management

**Fixed-lag smoothing memory usage:**
```
Memory ≈ (smootherLag / dt) × (size per state) × factors per state

Example:
  smootherLag = 5 seconds
  dt = 0.1 seconds (10 Hz)
  states = 50
  factors = ~3 per state
  Memory ≈ 50 × 3 × (pose + velocity + bias) ≈ 10-20 MB
```

### Debugging Performance Issues

#### Use ISAM2Result
```cpp
ISAM2Result result = isam.update(...);

std::cout << "Variables reeliminated: " 
          << result.variablesReeliminated << std::endl;
std::cout << "Cliques affected: " 
          << result.cliques << std::endl;
std::cout << "Error before: " << result.errorBefore.value() 
          << " after: " << result.errorAfter.value() << std::endl;
```

#### Check Timing
```cpp
auto start = std::chrono::high_resolution_clock::now();
isam.update(newFactors, newValues);
auto end = std::chrono::high_resolution_clock::now();

auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
std::cout << "Update took: " << duration.count() << " ms" << std::endl;
```

#### Profile Relinearization
```cpp
params.enableDetailedResults = true;
ISAM2Result result = isam.update(...);

// Check how many variables were relinearized
// High numbers = potential performance issue
```

---

## Expected Interactions with Kimera Integration

### What to Expect

#### 1. Initialization Phase
```
Kimera First Frame:
  ├─ Creates initial X(0), V(0), B(0)
  ├─ Adds PriorFactor with Kimera's initial estimate
  └─ ISAM2 stores this as linearization point
```

#### 2. Incremental Updates
```
Each Kimera Keyframe:
  ├─ Creates X(i), V(i), B(i) at keyframe timestamp
  ├─ Adds CombinedImuFactor from previous keyframe
  ├─ ISAM2 updates only new variables and connected ones
  └─ Returns optimized state back to Kimera
```

#### 3. Marginalization
```
When state exceeds smootherLag:
  ├─ IncrementalFixedLagSmoother identifies old states
  ├─ Orders them for elimination first
  ├─ ISAM2 marginalizes them into a linear factor
  └─ Memory stays constant, old states "absorbed" into prior
```

### Communication Flow

**Kimera → Your System:**
```cpp
// 1. Kimera provides timestamp and IMU data
adapter.addIMUMeasurement(timestamp, imu_data);

// 2. On keyframe, Kimera provides pose estimate  
adapter.addKeyframeState(timestamp, pose_estimate);

// 3. Kimera triggers optimization
adapter.optimizeGraph();
  └─> interface.optimize()
      └─> graph.constructFactorGraphFromTimestamps()
          └─> smoother.update(factors, values, timestamps)
              └─> ISAM2.update(factors, values)
```

**Your System → Kimera:**
```cpp
// Return optimized state
OptimizationResult result;
result.optimizedState = /* from ISAM2 */
result.optimizedVelocity = /* from ISAM2 */
result.optimizedBias = /* from ISAM2 */
result.covariance = /* marginal covariance */

return result;
```

### State Consistency

ISAM2 maintains consistency by:
1. **Joint optimization**: All states optimized together
2. **Covariance propagation**: Uncertainty properly tracked
3. **Marginalization**: Old states integrated as priors
4. **Relinearization**: Handles nonlinear motion

**Result**: Kimera gets globally consistent estimates even though it provides local/incremental data.

---

## Common Issues and Solutions

### Issue 1: Divergence

**Symptoms**: Error increases, estimates get worse
**Causes**: 
- Relinearization threshold too high
- Bad initial guesses
- Conflicting measurements

**Solutions**:
```cpp
// Lower relinearization threshold
params.relinearizeThreshold = 0.01;
params.relinearizeSkip = 1;

// Force relinearization
updateParams.force_relinearize = true;

// Better initial guesses (from Kimera predictions)
```

### Issue 2: Slow Performance

**Symptoms**: Updates take too long
**Causes**:
- Too many variables in window
- Excessive relinearization
- Dense factor connections

**Solutions**:
```cpp
// Reduce smoother lag
smootherLag = 2.0;  // instead of 5.0

// Less frequent relinearization
params.relinearizeSkip = 10;

// Use QR factorization for better sparsity
params.factorization = ISAM2Params::QR;
```

### Issue 3: Memory Growth

**Symptoms**: Memory usage increases over time
**Causes**:
- Not marginalizing old states
- Accumulating removed factors

**Solutions**:
```cpp
// Enable factor slot reuse
params.findUnusedFactorSlots = true;

// Ensure timestamps are properly set
timestamps[key] = current_time;

// Verify marginalization is happening
auto margKeys = smoother.findKeysBefore(current_time - lag);
```

---

## Further Reading

- **ISAM2 Paper**: "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (Kaess et al., 2012)
- **GTSAM Docs**: https://gtsam.org/
- **Factor Graphs**: "Factor Graphs for Robot Perception" (Dellaert & Kaess, 2017)

---

## Quick Reference

### Key Classes
```cpp
gtsam::ISAM2                          // Core solver
gtsam::ISAM2Params                    // Configuration
gtsam::ISAM2Result                    // Update result info
fgo::solvers::IncrementalFixedLagSmoother  // Your fixed-lag wrapper
```

### Essential Methods
```cpp
isam.update(factors, values)          // Add factors and optimize
isam.calculateEstimate()              // Get current solution
isam.calculateEstimate<Type>(key)     // Get specific variable
isam.marginalCovariance(key)          // Get uncertainty
smoother.update(factors, values, timestamps)  // Fixed-lag update
```

### Key Parameters
```cpp
relinearizeThreshold = 0.01-0.1       // When to relinearize
relinearizeSkip = 1-10                // How often to check
smootherLag = 1.0-5.0                 // Time window (seconds)
factorization = CHOLESKY|QR           // Linear solver method
```

---

**Next Steps**: Try examining the ISAM2Result from your optimization calls to see how many variables are being relinearized and affected. This will give you insight into the incremental behavior.
