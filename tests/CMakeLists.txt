# Tests for online_fgo_core
cmake_minimum_required(VERSION 3.16...3.26)

# Find GTest
find_package(GTest REQUIRED)
include(GoogleTest)

# GeographicLib is required by gtsam_unstable
find_package(GeographicLib REQUIRED)
if(GeographicLib_LIBRARY_DIRS)
  link_directories(${GeographicLib_LIBRARY_DIRS})
endif()

# Determine GeographicLib library to link
if(TARGET GeographicLib::GeographicLib_SHARED)
    set(GEOGRAPHICLIB_LIB GeographicLib::GeographicLib_SHARED)
elseif(TARGET GeographicLib::GeographicLib)
    set(GEOGRAPHICLIB_LIB GeographicLib::GeographicLib)
elseif(GeographicLib_LIBRARIES)
    set(GEOGRAPHICLIB_LIB ${GeographicLib_LIBRARIES})
else()
    message(FATAL_ERROR "GeographicLib not found properly")
endif()

# Common test dependencies
# Only link interface libraries that tests actually use
# (graph and integrator have dependencies on parent package)
set(TEST_LINK_LIBRARIES
    online_fgo_interface
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
)

set(TEST_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${GTEST_INCLUDE_DIRS}
    ${ONLINEFGO_CORE_INCLUDE}
)

# Kimera Integration Tests (conditional on ENABLE_KIMERA_INTEGRATION)
if(ENABLE_KIMERA_INTEGRATION)
    message(STATUS "Adding Kimera integration tests")
    
    # Test: Simple Kimera Integration (simulates Kimera-VIO input)
    add_executable(test_kimera_simple_integration test_kimera_simple_integration.cpp)
    target_include_directories(test_kimera_simple_integration PRIVATE ${TEST_INCLUDE_DIRS})
    target_link_libraries(test_kimera_simple_integration PRIVATE 
        ${TEST_LINK_LIBRARIES}
        online_fgo_graph
        online_fgo_data
        online_fgo_factors
        online_fgo_solvers
        online_fgo_utils
        online_fgo_integrator  
        gtsam
        gtsam_unstable
        ${GEOGRAPHICLIB_LIB}
    )
    gtest_discover_tests(test_kimera_simple_integration DISCOVERY_MODE PRE_TEST)
endif()

# Add all tests to a custom target for convenience
set(CORE_TEST_TARGETS
)

if(ENABLE_KIMERA_INTEGRATION)
    list(APPEND CORE_TEST_TARGETS
        test_kimera_simple_integration
    )
endif()

add_custom_target(run_core_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS ${CORE_TEST_TARGETS}
    COMMENT "Running all online_fgo_core tests"
)

message(STATUS "online_fgo_core tests configured:")
if(ENABLE_KIMERA_INTEGRATION)
    message(STATUS "  - test_kimera_simple_integration")
endif()
message(STATUS "Run tests with: make test or make run_core_tests")
