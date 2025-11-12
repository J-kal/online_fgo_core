# online_fgo_core Tests

This directory contains unit tests and integration tests for the ROS-agnostic `online_fgo_core` library.

## Test Files

### Interface Tests

- **test_logger_interface.cpp** - Tests for the LoggerInterface abstraction
  - Verifies info, warn, error, debug logging
  - Tests stream-style logging
  - Validates debug enable/disable functionality

- **test_parameter_interface.cpp** - Tests for the ParameterInterface abstraction
  - Tests parameter set/get for double, int, bool, string types
  - Validates default value handling
  - Tests parameter existence checking
  - Verifies nested parameter name support

- **test_time_interface.cpp** - Tests for the TimeInterface abstraction
  - Tests time construction and conversion
  - Validates time arithmetic operations
  - Tests comparison operators
  - Verifies nanosecond precision

- **test_application_interface.cpp** - Tests for the ApplicationInterface
  - Tests standalone application creation
  - Validates logger, parameter, and time access
  - Tests multiple application instances
  - Validates YAML parameter loading (if available)

- **test_equivalence.cpp** - Cross-implementation equivalence tests
  - Verifies parameter consistency across instances
  - Tests time and logger interface consistency
  - Validates complex parameter hierarchies
  - Includes performance benchmarks

## Test Configuration

- **test_params.yaml** - Sample parameter file for testing
  - Contains typical FGO configuration
  - Used for testing parameter loading

## Building Tests

```bash
cd online_fgo_core
mkdir -p build && cd build
cmake -DBUILD_TESTS=ON ..
make -j$(nproc)
```

## Running Tests

Run all tests:
```bash
make test
# Or with verbose output
ctest --output-on-failure
```

Run specific test:
```bash
./tests/test_logger_interface
./tests/test_parameter_interface
./tests/test_time_interface
./tests/test_application_interface
./tests/test_equivalence
```

Run tests with custom target:
```bash
make run_core_tests
```

## Test Requirements

- **GTest** - Google Test framework
- **online_fgo_core** - Core library with all dependencies (GTSAM, Eigen, etc.)
- **yaml-cpp** (optional) - For YAML parameter loading tests

## Expected Results

All tests should pass. Example output:

```
[==========] Running 45 tests from 5 test suites.
[----------] Global test environment set-up.
[----------] 12 tests from LoggerInterfaceTest
[ RUN      ] LoggerInterfaceTest.InfoLogging
[       OK ] LoggerInterfaceTest.InfoLogging (0 ms)
...
[----------] 12 tests from LoggerInterfaceTest (2 ms total)

[==========] 45 tests from 5 test suites ran. (15 ms total)
[  PASSED  ] 45 tests.
```

## Adding New Tests

1. Create a new test file: `test_new_feature.cpp`
2. Add to CMakeLists.txt:
   ```cmake
   add_executable(test_new_feature test_new_feature.cpp)
   target_include_directories(test_new_feature PRIVATE ${TEST_INCLUDE_DIRS})
   target_link_libraries(test_new_feature PRIVATE ${TEST_LINK_LIBRARIES})
   gtest_discover_tests(test_new_feature)
   ```
3. Add to `run_core_tests` target dependencies

## Continuous Integration

These tests are designed to run without ROS dependencies, making them suitable for CI/CD pipelines.

See the main [TESTING_GUIDE.md](../../TESTING_GUIDE.md) for more details.
