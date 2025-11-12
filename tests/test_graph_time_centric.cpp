/**
 * @file test_graph_time_centric.cpp
 * @brief Unit tests for GraphTimeCentric class
 * @author AI Assistant
 * @date 2025-10-24
 */

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include "online_fgo_core/graph/GraphTimeCentric.h"
#include "online_fgo_core/interface/ApplicationInterface.h"
#include "online_fgo_core/data/DataTypesFGO.h"
#include "online_fgo_core/data/DataTypesMeasurement.h"

using namespace fgo::graph;
using namespace fgo::core;
using namespace fgo::data;

/**
 * @brief Test fixture for GraphTimeCentric tests
 */
class GraphTimeCentricTest : public ::testing::Test {
protected:
    std::shared_ptr<StandaloneApplication> app;
    std::shared_ptr<GraphTimeCentric> graph;
    
    void SetUp() override {
        app = std::make_shared<StandaloneApplication>("test_time_centric");
        // Note: ConsoleLogger doesn't have setLevel, so we accept default logging
        graph = std::make_shared<GraphTimeCentric>(*app);
    }
    
    void TearDown() override {
        graph.reset();
        app.reset();
    }
    
    // Helper function to create a valid IMU measurement
    IMUMeasurement createIMUMeasurement(double timestamp) {
        IMUMeasurement imu;
        imu.timestamp = TimeStamp(timestamp, 0);
        imu.gyro = gtsam::Vector3(0.01, 0.02, 0.03);
        imu.accLin = gtsam::Vector3(0.1, 0.2, 9.81);
        imu.accRot = gtsam::Vector3(0.01, 0.02, 0.03);
        imu.accLinCov = gtsam::Matrix33::Identity() * 0.01;
        imu.accRotCov = gtsam::Matrix33::Identity() * 0.01;
        return imu;
    }
    
    // Helper function to create initial state
    State createInitialState(double timestamp) {
        State state;
        state.timestamp = TimeStamp(timestamp, 0);
        state.state = gtsam::NavState(
            gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
            gtsam::Vector3(0, 0, 0)
        );
        state.imuBias = gtsam::imuBias::ConstantBias();
        state.poseVar = gtsam::Matrix66::Identity() * 0.1;
        state.velVar = gtsam::Matrix33::Identity() * 0.01;
        state.imuBiasVar = gtsam::Matrix66::Identity() * 0.001;
        return state;
    }
};

// ============================================================================
// Construction and Initialization Tests
// ============================================================================

TEST_F(GraphTimeCentricTest, ConstructorCreatesValidGraph) {
    ASSERT_NE(graph, nullptr);
    EXPECT_EQ(graph->size(), 0);
}

TEST_F(GraphTimeCentricTest, InitializeGraphWithState) {
    State initState = createInitialState(1000.0);
    
    auto preIntParams = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0, 0, -9.81)
    );
    
    ASSERT_NO_THROW({
        graph->initGraph(initState, 1000.0, preIntParams);
    });
}

TEST_F(GraphTimeCentricTest, InitializeWithISAM2Solver) {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    params.factorization = gtsam::ISAM2Params::CHOLESKY;
    
    ASSERT_NO_THROW({
        graph->initSolver(params);
    });
}

// ============================================================================
// IMU-Based Graph Construction Tests
// ============================================================================

TEST_F(GraphTimeCentricTest, ConstructGraphOnIMU_EmptyData) {
    std::vector<IMUMeasurement> imuData;
    
    auto status = graph->constructFactorGraphOnIMU(imuData);
    // Empty data should not cause crash, but may return different status
    EXPECT_TRUE(status == StatusGraphConstruction::SUCCESSFUL ||
                status == StatusGraphConstruction::NO_OPTIMIZATION ||
                status == StatusGraphConstruction::PASSED);
}

TEST_F(GraphTimeCentricTest, ConstructGraphOnIMU_SingleMeasurement) {
    std::vector<IMUMeasurement> imuData;
    imuData.push_back(createIMUMeasurement(1000.0));
    
    auto status = graph->constructFactorGraphOnIMU(imuData);
    EXPECT_TRUE(status == StatusGraphConstruction::SUCCESSFUL ||
                status == StatusGraphConstruction::NO_OPTIMIZATION ||
                status == StatusGraphConstruction::PASSED);
}

TEST_F(GraphTimeCentricTest, ConstructGraphOnIMU_MultipleMeasurements) {
    std::vector<IMUMeasurement> imuData;
    for (int i = 0; i < 10; ++i) {
        imuData.push_back(createIMUMeasurement(1000.0 + i * 0.01));
    }
    
    auto status = graph->constructFactorGraphOnIMU(imuData);
    EXPECT_TRUE(status == StatusGraphConstruction::SUCCESSFUL ||
                status == StatusGraphConstruction::NO_OPTIMIZATION ||
                status == StatusGraphConstruction::PASSED);
}

TEST_F(GraphTimeCentricTest, ConstructGraphOnIMU_OutOfOrderMeasurements) {
    std::vector<IMUMeasurement> imuData;
    imuData.push_back(createIMUMeasurement(1000.1));
    imuData.push_back(createIMUMeasurement(1000.0));  // Out of order
    imuData.push_back(createIMUMeasurement(1000.2));
    
    // Should handle gracefully (sort or reject)
    ASSERT_NO_THROW({
        graph->constructFactorGraphOnIMU(imuData);
    });
}

// ============================================================================
// Time-Based Graph Construction Tests
// ============================================================================

TEST_F(GraphTimeCentricTest, ConstructGraphOnTime_EmptyTimestamps) {
    std::vector<double> timestamps;
    std::vector<IMUMeasurement> imuData;
    
    auto status = graph->constructFactorGraphOnTime(timestamps, imuData);
    EXPECT_TRUE(status == StatusGraphConstruction::SUCCESSFUL ||
                status == StatusGraphConstruction::NO_OPTIMIZATION ||
                status == StatusGraphConstruction::PASSED);
}

TEST_F(GraphTimeCentricTest, ConstructGraphOnTime_SingleTimestamp) {
    std::vector<double> timestamps = {1000.0};
    std::vector<IMUMeasurement> imuData;
    
    auto status = graph->constructFactorGraphOnTime(timestamps, imuData);
    EXPECT_TRUE(status == StatusGraphConstruction::SUCCESSFUL ||
                status == StatusGraphConstruction::NO_OPTIMIZATION ||
                status == StatusGraphConstruction::PASSED);
}

TEST_F(GraphTimeCentricTest, ConstructGraphOnTime_RegularIntervals) {
    std::vector<double> timestamps;
    for (int i = 0; i < 10; ++i) {
        timestamps.push_back(1000.0 + i * 0.1);
    }
    
    std::vector<IMUMeasurement> imuData;
    for (int i = 0; i < 100; ++i) {
        imuData.push_back(createIMUMeasurement(1000.0 + i * 0.01));
    }
    
    auto status = graph->constructFactorGraphOnTime(timestamps, imuData);
    EXPECT_TRUE(status == StatusGraphConstruction::SUCCESSFUL ||
                status == StatusGraphConstruction::NO_OPTIMIZATION ||
                status == StatusGraphConstruction::PASSED);
}

TEST_F(GraphTimeCentricTest, ConstructGraphOnTime_IrregularIntervals) {
    std::vector<double> timestamps = {1000.0, 1000.05, 1000.15, 1000.18, 1000.3};
    std::vector<IMUMeasurement> imuData;
    
    for (int i = 0; i < 50; ++i) {
        imuData.push_back(createIMUMeasurement(1000.0 + i * 0.01));
    }
    
    auto status = graph->constructFactorGraphOnTime(timestamps, imuData);
    EXPECT_TRUE(status == StatusGraphConstruction::SUCCESSFUL ||
                status == StatusGraphConstruction::NO_OPTIMIZATION ||
                status == StatusGraphConstruction::PASSED);
}

// ============================================================================
// Optimization Tests
// ============================================================================

TEST_F(GraphTimeCentricTest, OptimizeWithoutInitialization) {
    State newState = createInitialState(1000.0);
    
    // Optimize without initialization should handle gracefully
    ASSERT_NO_THROW({
        double time = graph->optimize(newState);
        EXPECT_GE(time, 0.0);
    });
}

TEST_F(GraphTimeCentricTest, OptimizeAfterInitialization) {
    State initState = createInitialState(1000.0);
    
    auto preIntParams = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0, 0, -9.81)
    );
    
    graph->initGraph(initState, 1000.0, preIntParams);
    
    gtsam::ISAM2Params params;
    graph->initSolver(params);
    
    State newState = createInitialState(1001.0);
    
    double optimizationTime = graph->optimize(newState);
    EXPECT_GE(optimizationTime, 0.0);
}

TEST_F(GraphTimeCentricTest, OptimizePreservesStateTimestamp) {
    State newState = createInitialState(1000.5);
    TimeStamp originalTimestamp = newState.timestamp;
    
    graph->optimize(newState);
    
    // Timestamp should be preserved or updated consistently
    EXPECT_TRUE(newState.timestamp == originalTimestamp ||
                newState.timestamp > originalTimestamp);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(GraphTimeCentricTest, FullPipeline_InitConstructOptimize) {
    // 1. Initialize graph
    State initState = createInitialState(1000.0);
    auto preIntParams = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0, 0, -9.81)
    );
    graph->initGraph(initState, 1000.0, preIntParams);
    
    // 2. Initialize solver
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    graph->initSolver(params);
    
    // 3. Construct graph with IMU data
    std::vector<IMUMeasurement> imuData;
    for (int i = 0; i < 20; ++i) {
        imuData.push_back(createIMUMeasurement(1000.0 + i * 0.01));
    }
    
    auto constructStatus = graph->constructFactorGraphOnIMU(imuData);
    EXPECT_TRUE(constructStatus == StatusGraphConstruction::SUCCESSFUL ||
                constructStatus == StatusGraphConstruction::NO_OPTIMIZATION ||
                constructStatus == StatusGraphConstruction::PASSED);
    
    // 4. Optimize
    State newState = createInitialState(1000.2);
    double optimizationTime = graph->optimize(newState);
    EXPECT_GE(optimizationTime, 0.0);
}

TEST_F(GraphTimeCentricTest, SequentialOptimizations) {
    State initState = createInitialState(1000.0);
    auto preIntParams = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0, 0, -9.81)
    );
    graph->initGraph(initState, 1000.0, preIntParams);
    
    gtsam::ISAM2Params params;
    graph->initSolver(params);
    
    // Perform multiple sequential optimizations
    for (int i = 1; i <= 5; ++i) {
        std::vector<IMUMeasurement> imuData;
        for (int j = 0; j < 10; ++j) {
            imuData.push_back(createIMUMeasurement(1000.0 + (i-1) * 0.1 + j * 0.01));
        }
        
        graph->constructFactorGraphOnIMU(imuData);
        
        State newState = createInitialState(1000.0 + i * 0.1);
        double time = graph->optimize(newState);
        EXPECT_GE(time, 0.0);
    }
}

// ============================================================================
// Reset and Cleanup Tests
// ============================================================================

TEST_F(GraphTimeCentricTest, ResetGraphClearsState) {
    // Add some factors
    std::vector<IMUMeasurement> imuData;
    imuData.push_back(createIMUMeasurement(1000.0));
    graph->constructFactorGraphOnIMU(imuData);
    
    // Reset
    graph->resetGraph();
    
    // Graph should be empty
    EXPECT_EQ(graph->size(), 0);
}

TEST_F(GraphTimeCentricTest, DestructorCleanup) {
    // Create graph in local scope
    {
        auto localGraph = std::make_shared<GraphTimeCentric>(*app);
        State initState = createInitialState(1000.0);
        auto preIntParams = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
            gtsam::Vector3(0, 0, -9.81)
        );
        localGraph->initGraph(initState, 1000.0, preIntParams);
    }
    // Should destruct without issues
    SUCCEED();
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
