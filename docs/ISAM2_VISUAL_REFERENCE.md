# ISAM2 Visual Reference

## Factor Graph Evolution Over Time

### Time t=0: Initial State
```
States:
  X(0)  V(0)  B(0)
   │     │     │
Factors:
  └─────┴─────┘
  [PriorFactor]
  (From Kimera initial estimate)

Bayes Tree:
   Root
    │
  [X₀,V₀,B₀]

ISAM2 Status:
- Variables: 3
- Factors: 1
- Cliques: 1
```

### Time t=1: First IMU Integration
```
States:
  X(0)  V(0)  B(0)    X(1)  V(1)  B(1)
   │     │     │       │     │     │
Factors:
  └─────┴─────┘       │     │     │
  [PriorFactor]       │     │     │
         │            │     │     │
         └────────────┴─────┴─────┘
         [CombinedImuFactor]
         (Preintegrated IMU from t=0 to t=1)

Bayes Tree:
      Root
     /    \
[X₀,V₀,B₀] [X₁,V₁,B₁]

ISAM2 Status:
- Variables: 6
- Factors: 2
- Update type: Incremental (added 3 variables)
```

### Time t=2: Second IMU Integration
```
States:
  X(0)  V(0)  B(0)    X(1)  V(1)  B(1)    X(2)  V(2)  B(2)
   │     │     │       │     │     │       │     │     │
Factors:
  └─────┴─────┘       │     │     │       │     │     │
  [PriorFactor]       │     │     │       │     │     │
         │            │     │     │       │     │     │
         └────────────┴─────┴─────┘       │     │     │
         [CombinedImuFactor]              │     │     │
                   │                      │     │     │
                   └──────────────────────┴─────┴─────┘
                   [CombinedImuFactor]
                   (Preintegrated IMU from t=1 to t=2)

Bayes Tree:
           Root
          /  |  \
     [X₀] [X₁] [X₂]
      │    │    │
     [V₀] [V₁] [V₂]
      │    │    │
     [B₀] [B₁] [B₂]

ISAM2 Status:
- Variables: 9
- Factors: 3
- Update type: Incremental
```

### Time t=3: With GP Motion Prior (if enabled)
```
States:
  X(0)  V(0)  W(0)  B(0)    X(1)  V(1)  W(1)  B(1)    X(2)  V(2)  W(2)  B(2)
   │     │     │     │       │     │     │     │       │     │     │     │
Factors:
  └─────┴─────┴─────┘       │     │     │     │       │     │     │     │
  [PriorFactor]             │     │     │     │       │     │     │     │
         │                  │     │     │     │       │     │     │     │
         └──────────────────┴─────┴─────┴─────┘       │     │     │     │
         [CombinedImuFactor]                          │     │     │     │
         (IMU measurements t=0→t=1)                   │     │     │     │
                   │                                  │     │     │     │
                   └──────────────[GPPrior]───────────┘     │     │     │
                   │              (Smooth motion            │     │     │
                   │               X,V,W constrained)       │     │     │
                   │                                        │     │     │
                   └────────────────────────────────────────┴─────┴─────┘
                   [CombinedImuFactor]
                   (IMU measurements t=1→t=2)
                                │
                                └──────────────[GPPrior]───────────>
                                               (Smooth motion
                                                X,V,W constrained)

Key Observation:
- IMU Factor: Connects X, V, B (measurement constraint)
- GP Prior: Connects X, V, W (smoothness constraint)
- Both factors share X, V variables → Joint optimization!
- W (angular velocity) may be needed for full GP model

Dense connections = More constraint = Better accuracy
```

---

## GP Motion Prior Factor Graph Details

### Factor Graph Comparison

#### Without GP Priors (IMU Only)
```
Variables per state: 3 (X, V, B)
Factors between states: 1 (IMU)

State 0          State 1          State 2
┌─────────┐     ┌─────────┐     ┌─────────┐
│ X(0) ●  │     │ X(1) ●  │     │ X(2) ●  │
│ V(0) ●  │     │ V(1) ●  │     │ V(2) ●  │
│ B(0) ●  │     │ B(1) ●  │     │ B(2) ●  │
└─────────┘     └─────────┘     └─────────┘
     │               │               │
     └───[IMU 0→1]───┘               │
     (3 vars)                        │
                     └───[IMU 1→2]───┘
                     (3 vars)

Optimization Characteristics:
✓ Sparse: Each state connects to neighbors only
✓ Fast: O(k²) where k = vars per connection ≈ 6
✗ No smoothness enforcement
✗ Vulnerable to IMU noise spikes
```

#### With GP Priors Added
```
Variables per state: 4* (X, V, W, B)  *if using full GP model
Factors between states: 2 (IMU + GP)

State 0              State 1              State 2
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ X(0) ●──────┼─┐   │ X(1) ●──────┼─┐   │ X(2) ●      │
│ V(0) ●──────┼─┼─┐ │ V(1) ●──────┼─┼─┐ │ V(2) ●      │
│ W(0) ●──────┼─┼─┼─┤ W(1) ●──────┼─┼─┼─┤ W(2) ●      │
│ B(0) ●──────┼─┘ │ │ B(1) ●──────┼─┘ │ │ B(2) ●      │
└─────────────┘   │ └─────────────┘   │ └─────────────┘
     │  │         │      │  │         │
     │  └─[GP 0→1]┘      │  └─[GP 1→2]┘
     │   (smoothness)    │   (smoothness)
     │   X,V,W           │   X,V,W
     │                   │
     └────[IMU 0→1]──────┘
          (measurement)
          X,V,B,B

Optimization Characteristics:
✓ Denser: More inter-variable connections
✓ Smoother: GP enforces continuous acceleration
✓ More robust: GP filters IMU noise
✗ Slower: ~30-50% more computation
✗ More parameters: Q_c tuning needed

Shared Variables Create Information Fusion:
  X(0), V(0) appear in BOTH IMU and GP factors
  → Joint optimization balances both constraints
  → Solution minimizes: ||e_imu||² + ||e_gp||²
```

### Information Matrix Structure

#### Hessian Sparsity Pattern

**Without GP Priors:**
```
       X₀ V₀ B₀ X₁ V₁ B₁ X₂ V₂ B₂
  X₀ [ ●  ●  ·  ●  ·  ·  ·  ·  · ]  ← Prior + IMU₀₁
  V₀ [ ●  ●  ·  ●  ·  ·  ·  ·  · ]  ← Prior + IMU₀₁
  B₀ [ ·  ·  ●  ·  ·  ●  ·  ·  · ]  ← Prior + IMU₀₁
  X₁ [ ●  ●  ·  ●  ●  ·  ●  ·  · ]  ← IMU₀₁ + IMU₁₂
  V₁ [ ·  ·  ·  ●  ●  ·  ●  ·  · ]  ← IMU₀₁ + IMU₁₂
  B₁ [ ·  ·  ●  ·  ·  ●  ·  ·  ●  ]  ← IMU₀₁ + IMU₁₂
  X₂ [ ·  ·  ·  ●  ●  ·  ●  ●  · ]  ← IMU₁₂
  V₂ [ ·  ·  ·  ·  ·  ·  ●  ●  · ]  ← IMU₁₂
  B₂ [ ·  ·  ·  ·  ·  ●  ·  ·  ●  ]  ← IMU₁₂

Sparsity: 33/81 = 41% filled
Band structure: Tridiagonal blocks
```

**With GP Priors Added:**
```
       X₀ V₀ W₀ B₀ X₁ V₁ W₁ B₁ X₂ V₂ W₂ B₂
  X₀ [ ●  ●  ●  ·  ●  ●  ●  ·  ·  ·  ·  · ]  ← Prior + IMU₀₁ + GP₀₁
  V₀ [ ●  ●  ●  ·  ●  ●  ●  ·  ·  ·  ·  · ]  ← Prior + IMU₀₁ + GP₀₁
  W₀ [ ●  ●  ●  ·  ●  ●  ●  ·  ·  ·  ·  · ]  ← Prior + GP₀₁
  B₀ [ ·  ·  ·  ●  ·  ·  ·  ●  ·  ·  ·  · ]  ← Prior + IMU₀₁
  X₁ [ ●  ●  ●  ·  ●  ●  ●  ·  ●  ●  ●  · ]  ← IMU₀₁ + GP₀₁ + IMU₁₂ + GP₁₂
  V₁ [ ●  ●  ●  ·  ●  ●  ●  ·  ●  ●  ●  · ]  ← IMU₀₁ + GP₀₁ + IMU₁₂ + GP₁₂
  W₁ [ ●  ●  ●  ·  ●  ●  ●  ·  ●  ●  ●  · ]  ← GP₀₁ + GP₁₂
  B₁ [ ·  ·  ·  ●  ·  ·  ·  ●  ·  ·  ·  ●  ]  ← IMU₀₁ + IMU₁₂
  X₂ [ ·  ·  ·  ·  ●  ●  ●  ·  ●  ●  ●  · ]  ← IMU₁₂ + GP₁₂
  V₂ [ ·  ·  ·  ·  ·  ·  ·  ·  ●  ●  ●  · ]  ← IMU₁₂ + GP₁₂
  W₂ [ ·  ·  ·  ·  ●  ●  ●  ·  ●  ●  ●  · ]  ← GP₁₂
  B₂ [ ·  ·  ·  ·  ·  ·  ·  ●  ·  ·  ·  ●  ]  ← IMU₁₂

Sparsity: 72/144 = 50% filled
More filled = More coupling = Smoother solution
Band structure: Still tridiagonal but denser blocks
```

**Key Insight**: GP priors create **cross-coupling** between X, V, W variables that wouldn't exist with IMU only.

---

## GP Prior Effect on Trajectory

### Visual Comparison

#### Scenario: IMU has measurement spike at t=2

**IMU Measurements:**
```
t=0: acc=[0, 0, 0], gyro=[0, 0, 0]
t=1: acc=[0.1, 0, 0], gyro=[0, 0, 0.1]  ← normal
t=2: acc=[5.0, 2.0, 0], gyro=[1.0, 0, 0]  ← SPIKE (sensor glitch)
t=3: acc=[0.1, 0, 0], gyro=[0, 0, 0.1]  ← back to normal
t=4: acc=[0.1, 0, 0], gyro=[0, 0, 0.1]  ← normal
```

**Trajectory Without GP Priors (IMU Only):**
```
Y-axis (position)
  │
3 │                    ●  ← X(2) large jump
  │                   /│\
2 │                  / │ \
  │                 /  │  \
1 │     ●──────────●   │   ●──────●  ← X(0), X(1), X(3), X(4)
  │     
0 └────────────────────────────────────> X-axis
    t=0  t=1         t=2  t=3  t=4

Characteristics:
- Follows IMU measurements exactly
- Large discontinuity at t=2
- Physically unrealistic (instantaneous acceleration)
- No smoothing of glitches
```

**Trajectory With GP Priors (Q_c = moderate):**
```
Y-axis (position)
  │
3 │                    
  │                   
2 │                      ●  ← X(2) partially smoothed
  │                     ╱ ╲
1 │     ●──────────●──●───●─●  ← Smooth curve
  │                         
0 └────────────────────────────────────> X-axis
    t=0  t=1    t=2  t=3  t=4

Characteristics:
- Balances IMU data with smoothness
- Reduced spike at t=2 (GP resists sudden change)
- Smoother overall trajectory
- More physically realistic
```

**Trajectory With GP Priors (Q_c = tight):**
```
Y-axis (position)
  │
3 │                    
  │                   
2 │                    
  │                   
1 │     ●──────────●──●───●──●  ← Very smooth
  │                         
0 └────────────────────────────────────> X-axis
    t=0  t=1    t=2  t=3  t=4

Characteristics:
- Strong smoothing (small Q_c)
- Almost completely rejects spike
- May under-fit true motion changes
- Very smooth but potentially inaccurate for real maneuvers
```

### Acceleration Profiles

```
Acceleration (m/s²)
  │
5 │         ┌─┐                    ← Without GP (follows spike)
4 │         │ │
3 │         │ │
2 │         │ │        ┌─┐         ← With GP (smoothed)
1 │    ┌────┘ └────┬───┘ └───┐
0 ├────┘           └──────────────┘
  └────────────────────────────────> Time
     t=0  t=1  t=2  t=3  t=4

GP Prior Effect:
- Penalizes sudden acceleration changes
- Creates smoother acceleration profile
- Acts as low-pass filter on IMU data
- Filter strength controlled by Q_c
```

### Time t=5: Fixed-Lag Marginalization (smootherLag = 3.0s)
```
Current time: t=5.0s
Smoother lag: 3.0s
Keep states from: t=2.0s onwards

States to marginalize: X(0), V(0), B(0)

Before Marginalization:
  X(0)  V(0)  B(0)    X(1)  V(1)  B(1)    X(2)  V(2)  B(2)    ...  X(5)  V(5)  B(5)
   │     │     │       │     │     │       │     │     │            │     │     │
  [PriorFactor]       │     │     │       │     │     │            │     │     │
         │            │     │     │       │     │     │            │     │     │
         └────────────┴─────┴─────┘       │     │     │            │     │     │
         [CombinedImuFactor]              │     │     │            │     │     │
                   ↑                      │     │     │            │     │     │
                   │                      │     │     │            │     │     │
              TO BE MARGINALIZED         │     │     │            │     │     │
              (t < 2.0s)                 │     │     │            │     │     │

After Marginalization:
  [LinearFactor]      X(1)  V(1)  B(1)    X(2)  V(2)  B(2)    ...  X(5)  V(5)  B(5)
   (Absorbs X₀,V₀,B₀)  │     │     │       │     │     │            │     │     │
         └─────────────┴─────┴─────┘       │     │     │            │     │     │
         (Now a prior on X₁,V₁,B₁)         │     │     │            │     │     │
                                            │     │     │            │     │     │
                                    [CombinedImuFactor]              │     │     │
                                                 ...         [CombinedImuFactor]

Result:
- Memory freed from X(0), V(0), B(0)
- Information preserved in linear prior on X(1), V(1), B(1)
- Constant memory usage over time!
```

---

## Relinearization Process

### When Does Relinearization Happen?

```
Update Counter:  0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20
                 │                 │                 │                 │
Check Relin:     └─────────────────┴─────────────────┴─────────────────┴─────>
                 (every relinearizeSkip=10 updates)

For each variable:
  δ = ||θ_current - θ_linearization||
  
  If δ > relinearizeThreshold:
    ├─ Relinearize this variable
    ├─ Propagate to connected variables ("wildfire")
    └─ Update affected Bayes Tree cliques
```

### Relinearization Example

```
State Evolution:

t=0: θ₀ = [x=0, y=0, θ=0]
     Linearization point: θ₀
     δ = 0

t=1: θ₁ = [x=0.1, y=0.05, θ=0.02]
     Linearization point: θ₀
     δ = 0.11 > 0.1 (threshold)
     → RELINEARIZE at θ₁

t=2: θ₂ = [x=0.15, y=0.08, θ=0.03]
     Linearization point: θ₁
     δ = 0.06 < 0.1
     → No relinearization

t=3: θ₃ = [x=0.21, y=0.12, θ=0.05]
     Linearization point: θ₁
     δ = 0.12 > 0.1
     → RELINEARIZE at θ₃
```

### Wildfire Propagation

```
Initial state: Variable X(5) needs relinearization

Bayes Tree Before:
         Root
        /    \
      X(3)   X(5)*  ← needs relinearization
      /  \     |
    V(3) B(3) V(5)*  ← connected, mark for relin
                |
               B(5)*  ← connected, mark for relin

Bayes Tree After Relinearization:
         Root
        /    \
      X(3)   X(5)†  ← relinearized
      /  \     |
    V(3) B(3) V(5)†  ← relinearized
                |
               B(5)†  ← relinearized

Legend:
  * = marked for relinearization
  † = relinearized
```

---

## Memory Layout Over Time

### Without Fixed-Lag Smoothing (Full SLAM)

```
Time:     t=0    t=1    t=2    t=3    t=4    t=5
         ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐
States:  │ 3 │  │ 6 │  │ 9 │  │12 │  │15 │  │18 │  variables
         └───┘  └───┘  └───┘  └───┘  └───┘  └───┘
Memory:   10MB   20MB   30MB   40MB   50MB   60MB
                 ↗↗↗ LINEAR GROWTH ↗↗↗
```

### With Fixed-Lag Smoothing (lag=3s)

```
Time:     t=0    t=1    t=2    t=3    t=4    t=5
         ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐
States:  │ 3 │  │ 6 │  │ 9 │  │ 9 │  │ 9 │  │ 9 │  variables
         └───┘  └───┘  └───┘  └───┘  └───┘  └───┘
Memory:   10MB   20MB   30MB   30MB   30MB   30MB
                 ↗↗↗            ─ CONSTANT ─
                
At t=3: Marginalize states older than t-3=0
At t=4: Marginalize states older than t-3=1
At t=5: Marginalize states older than t-3=2
```

---

## Optimization Complexity Comparison

### Batch (LM/GN)

```
Every Update:
  ├─ Build full Hessian: n×n matrix (n = all variables)
  ├─ Solve linear system: O(n³)
  └─ Update all variables

Example with 100 states:
  300 variables × 300 = 90,000 matrix
  Operations: ~27 million
  Time: 500-1000 ms
```

### ISAM2 Incremental

```
New Data Arrives:
  ├─ Add new variables: m (typically 3-10)
  ├─ Identify affected: k (typically 10-30)
  ├─ Update subtree: O(k³)
  └─ Reuse rest of tree

Example with 100 states, adding 1:
  New: 3 variables
  Affected: 20 variables
  Operations: ~8,000
  Time: 10-50 ms
  
Speedup: 100-1000x faster!
```

---

## Factor Graph Sparsity Pattern

### Visual Representation

Each row/column = one variable
Black dot = non-zero entry in Hessian

#### Dense Problem (Bad)
```
     X₀ V₀ B₀ X₁ V₁ B₁ X₂ V₂ B₂
X₀ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]
V₀ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]
B₀ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]
X₁ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]
V₁ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]
B₁ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]
X₂ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]
V₂ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]
B₂ [ ●  ●  ●  ●  ●  ●  ●  ●  ● ]

Sparsity: 0% → Very slow
```

#### Sparse Problem (Good - Your Case)
```
     X₀ V₀ B₀ X₁ V₁ B₁ X₂ V₂ B₂
X₀ [ ●  ●  ●  ·  ·  ·  ·  ·  · ]
V₀ [ ●  ●  ●  ·  ·  ·  ·  ·  · ]
B₀ [ ●  ●  ●  ●  ●  ●  ·  ·  · ]
X₁ [ ·  ·  ●  ●  ●  ●  ·  ·  · ]
V₁ [ ·  ·  ●  ●  ●  ●  ·  ·  · ]
B₁ [ ·  ·  ●  ●  ●  ●  ●  ●  ● ]
X₂ [ ·  ·  ·  ·  ·  ●  ●  ●  ● ]
V₂ [ ·  ·  ·  ·  ·  ●  ●  ●  ● ]
B₂ [ ·  ·  ·  ·  ·  ●  ●  ●  ● ]

Sparsity: ~70% → Very fast!

Note: Band structure from temporal ordering
```

---

## Kimera Integration Data Flow

### Detailed Sequence Diagram

```
Kimera           Adapter          Interface         Graph            Smoother         ISAM2
  │                │                 │                │                 │               │
  │  IMU data      │                 │                │                 │               │
  ├───────────────>│ buffer          │                │                 │               │
  │                │                 │                │                 │               │
  │  Keyframe      │                 │                │                 │               │
  ├───────────────>│                 │                │                 │               │
  │                │ createState(t)  │                │                 │               │
  │                ├────────────────>│                │                 │               │
  │                │                 │ findOrCreate   │                 │               │
  │                │                 ├───────────────>│                 │               │
  │                │                 │   state_idx    │                 │               │
  │                │                 │<───────────────┤                 │               │
  │                │  StateHandle    │                │                 │               │
  │                │<────────────────┤                │                 │               │
  │                │                 │                │                 │               │
  │  Optimize      │                 │                │                 │               │
  ├───────────────>│                 │                │                 │               │
  │                │ optimize()      │                │                 │               │
  │                ├────────────────>│                │                 │               │
  │                │                 │ construct      │                 │               │
  │                │                 │ factor graph   │                 │               │
  │                │                 ├───────────────>│ factors         │               │
  │                │                 │                │ values          │               │
  │                │                 │                │ timestamps      │               │
  │                │                 │                ├────────────────>│               │
  │                │                 │                │                 │ update()      │
  │                │                 │                │                 ├──────────────>│
  │                │                 │                │                 │               │
  │                │                 │                │                 │  [Incremental │
  │                │                 │                │                 │   Bayes Tree  │
  │                │                 │                │                 │   Update]     │
  │                │                 │                │                 │               │
  │                │                 │                │                 │  [Wildfire]   │
  │                │                 │                │                 │               │
  │                │                 │                │                 │  [Marginalize]│
  │                │                 │                │                 │               │
  │                │                 │                │                 │<──────────────│
  │                │                 │                │  result         │               │
  │                │                 │                │<────────────────┤               │
  │                │                 │   optimized    │                 │               │
  │                │                 │   state        │                 │               │
  │                │                 │<───────────────┤                 │               │
  │                │  OptResult      │                │                 │               │
  │                │<────────────────┤                │                 │               │
  │  NavState      │                 │                │                 │               │
  │<───────────────┤                 │                │                 │               │
  │  bias          │                 │                │                 │               │
  │<───────────────┤                 │                │                 │               │
  │                │                 │                │                 │               │
```

### Key Insight: Why ISAM2 is Perfect Here

1. **Temporal Structure**: Sequential states → sparse factor graph
2. **Incremental Data**: New keyframes arrive one at a time
3. **Fixed Window**: Old data marginalized → constant complexity
4. **Real-time**: Fast updates required → ISAM2's strength

Your integration leverages all of ISAM2's advantages!

---

## Quick Diagnostic Checklist

### Is ISAM2 Working Properly?

✅ **Check These:**

```cpp
// 1. Variables are being added
ISAM2Result result = isam.update(...);
assert(result.newFactorsIndices.size() > 0);

// 2. Error is decreasing or stable
LOG(INFO) << "Error: " << result.errorAfter.value();
// Should be < 1e6 typically

// 3. Relinearization happening occasionally
LOG(INFO) << "Variables relinearized: " 
          << result.variablesRelinearized;
// Should be > 0 every ~10 updates

// 4. Marginalization working (fixed-lag)
size_t numVars = isam.getLinearizationPoint().size();
LOG(INFO) << "Active variables: " << numVars;
// Should stabilize, not grow indefinitely

// 5. Covariances are reasonable
Matrix cov = marginals.marginalCovariance(X(i));
LOG(INFO) << "Pose uncertainty: " << cov.norm();
// Should be reasonable (not NaN, not huge)
```

❌ **Warning Signs:**

- Error increasing over time → divergence
- No relinearization ever → check threshold
- Memory growing unbounded → marginalization not working
- NaN/Inf in results → numerical issues, check factor noise models
- Very slow updates → too many variables, reduce window

---

**For More Details**: See `ISAM2_TUTORIAL.md` for comprehensive theory and examples.
