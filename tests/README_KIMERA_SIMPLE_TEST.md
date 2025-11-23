# Simple Kimera Integration Test

This test simulates input from Kimera-VIO and verifies that the factor graph can be built timized.

## What the Test Does

1. **Creates 5 keyframes** at 0.1s intervals (simulating camera keyframes)
2. **Creates PIM data** (Preintegrated IMU Measurements) between consecutive keyframes
3. **Adds states and PIM data** to the factor graph
4. **Optimizes the graph** using GTSAM
5. **Verifies** that optimization succeeds and returns reasonable results

## Building the Test

The test is automatically included when `ENABLE_KIMERA_INTEGRATION` is enabled.

### Build with catkin:

```bash
cd /workspaces/src
catkin build online_fgo_core --cmake-args -DENABLE_KIMERA_INTEGRATION=ON
```

Or if already configured:

```bash
catkin build online_fgo_core
```

## Running the Test

### Option 1: Run all tests

```bash
cd /workspaces/src
catkin test online_fgo_core
```

### Option 2: Run only this test

```bash
cd /workspaces/src/build/online_fgo_core
ctest -R test_kimera_simple_integration -V
```

### Option 3: Run directly

```bash
cd /workspaces/src/build/online_fgo_core/tests
./test_kimera_simple_integration
```

### Option 4: Run with gtest filter (run specific test case)

```bash
cd /workspaces/src/build/online_fgo_core/tests
./test_kimera_simple_integration --gtest_filter=KimeraSimpleIntegration.BuildAndOptimizeGraph
```

## Expected Output

When the test runs successfully, you should see:

```
[==========] Running 2 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 2 tests from KimeraSimpleIntegration
[ RUN      ] KimeraSimpleIntegration.BuildAndOptimizeGraph
Optimization successful!
  Iterations: X
  Final error: Y
  Converged: yes/no
State 0 optimized pose: ...
State 1 optimized pose: ...
...
[       OK ] KimeraSimpleIntegration.BuildAndOptimizeGraph (XXX ms)
[ RUN      ] KimeraSimpleIntegration.MinimalTwoKeyframes
Minimal test - Iterations: X, Error: Y
[       OK ] KimeraSimpleIntegration.MinimalTwoKeyframes (XXX ms)
[----------] 2 tests from KimeraSimpleIntegration (XXX ms total)
[==========] All tests passed!
```

## Test Cases

### 1. `BuildAndOptimizeGraph`
- Creates 5 keyframes with simulated motion (forward at 1 m/s)
- Creates PIM data with realistic IMU measurements
- Verifies optimization succeeds
- Checks that optimized poses can be retrieved

### 2. `MinimalTwoKeyframes`
- Minimal test with only 2 keyframes
- Simple PIM with single IMU measurement
- Verifies basic functionality works

## Troubleshooting

### Test fails to compile
- Ensure `ENABLE_KIMERA_INTEGRATION=ON` is set
- Check that all dependencies are built (GTSAM, online_fgo_core libraries)

### Test fails at runtime
- Check that GTSAM is properly installed
- Verify that the integration interface initializes correctly
- Check logs for specific error messages

### Optimization fails
- This might indicate an issue with the factor graph construction
- Check that PIM data is being added correctly
- Verify that states are created with valid initial values

## Modifying the Test

To modify the test parameters:

1. **Number of keyframes**: Change `num_keyframes` in `BuildAndOptimizeGraph`
2. **Time interval**: Change `dt` (currently 0.1s)
3. **IMU measurements**: Modify the `acc_measured` and `gyro_measured` values
4. **Motion model**: Change the pose creation to simulate different motion patterns

