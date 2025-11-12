# Tests for online_fgo_core
cmake_minimum_required(VERSION 3.16...3.26)

# Find GTest
find_package(GTest REQUIRED)
include(GoogleTest)

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

# Test: Logger Interface
add_executable(test_logger_interface test_logger_interface.cpp)
target_include_directories(test_logger_interface PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_logger_interface PRIVATE ${TEST_LINK_LIBRARIES})
gtest_discover_tests(test_logger_interface)

# Test: Parameter Interface
add_executable(test_parameter_interface test_parameter_interface.cpp)
target_include_directories(test_parameter_interface PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_parameter_interface PRIVATE ${TEST_LINK_LIBRARIES})
gtest_discover_tests(test_parameter_interface)

# Test: Time Interface
add_executable(test_time_interface test_time_interface.cpp)
target_include_directories(test_time_interface PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_time_interface PRIVATE ${TEST_LINK_LIBRARIES})
gtest_discover_tests(test_time_interface)

# Test: Application Interface
add_executable(test_application_interface test_application_interface.cpp)
target_include_directories(test_application_interface PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_application_interface PRIVATE ${TEST_LINK_LIBRARIES})
gtest_discover_tests(test_application_interface)

# Test: Equivalence (verifies all implementations work the same)
add_executable(test_equivalence test_equivalence.cpp)
target_include_directories(test_equivalence PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_equivalence PRIVATE ${TEST_LINK_LIBRARIES})
gtest_discover_tests(test_equivalence)

# Test: Graph Base
add_executable(test_graph_base test_graph_base.cpp)
target_include_directories(test_graph_base PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_graph_base PRIVATE 
    ${TEST_LINK_LIBRARIES}
    online_fgo_graph
    online_fgo_data
    online_fgo_factors
    online_fgo_solvers
    online_fgo_utils
    gtsam
    gtsam_unstable
)
gtest_discover_tests(test_graph_base DISCOVERY_MODE PRE_TEST)

# Test: Graph Time Centric
add_executable(test_graph_time_centric test_graph_time_centric.cpp)
target_include_directories(test_graph_time_centric PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_graph_time_centric PRIVATE 
    ${TEST_LINK_LIBRARIES}
    online_fgo_graph
    online_fgo_data
    online_fgo_factors
    online_fgo_solvers
    online_fgo_utils
    gtsam
    gtsam_unstable
)
gtest_discover_tests(test_graph_time_centric DISCOVERY_MODE PRE_TEST)

# Kimera Integration Tests (conditional on ENABLE_KIMERA_INTEGRATION)
if(ENABLE_KIMERA_INTEGRATION)
    message(STATUS "Adding Kimera integration tests")
    
    # Test: KimeraIntegrationInterface
    add_executable(test_kimera_integration_interface test_kimera_integration_interface.cpp)
    target_include_directories(test_kimera_integration_interface PRIVATE ${TEST_INCLUDE_DIRS})
    target_link_libraries(test_kimera_integration_interface PRIVATE 
        ${TEST_LINK_LIBRARIES}
        online_fgo_graph
        online_fgo_data
        online_fgo_factors
        online_fgo_solvers
        online_fgo_utils
        online_fgo_integration  # Kimera integration library
        gtsam
        gtsam_unstable
    )
    gtest_discover_tests(test_kimera_integration_interface DISCOVERY_MODE PRE_TEST)
    
    # Test: GraphTimeCentricKimera
    add_executable(test_graph_time_centric_kimera test_graph_time_centric_kimera.cpp)
    target_include_directories(test_graph_time_centric_kimera PRIVATE ${TEST_INCLUDE_DIRS})
    target_link_libraries(test_graph_time_centric_kimera PRIVATE 
        ${TEST_LINK_LIBRARIES}
        online_fgo_graph
        online_fgo_data
        online_fgo_factors
        online_fgo_solvers
        online_fgo_utils
        online_fgo_integration
        gtsam
        gtsam_unstable
    )
    gtest_discover_tests(test_graph_time_centric_kimera DISCOVERY_MODE PRE_TEST)
endif()

# Add all tests to a custom target for convenience
set(CORE_TEST_TARGETS
    test_logger_interface
    test_parameter_interface
    test_time_interface
    test_application_interface
    test_equivalence
    test_graph_base
    test_graph_time_centric
)

if(ENABLE_KIMERA_INTEGRATION)
    list(APPEND CORE_TEST_TARGETS
        test_kimera_integration_interface
        test_graph_time_centric_kimera
    )
endif()

add_custom_target(run_core_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS ${CORE_TEST_TARGETS}
    COMMENT "Running all online_fgo_core tests"
)

message(STATUS "online_fgo_core tests configured:")
message(STATUS "  - test_logger_interface")
message(STATUS "  - test_parameter_interface")
message(STATUS "  - test_time_interface")
message(STATUS "  - test_application_interface")
message(STATUS "  - test_equivalence")
message(STATUS "  - test_graph_base")
message(STATUS "  - test_graph_time_centric")
if(ENABLE_KIMERA_INTEGRATION)
    message(STATUS "  - test_kimera_integration_interface")
    message(STATUS "  - test_graph_time_centric_kimera")
endif()
message(STATUS "Run tests with: make test or make run_core_tests")
