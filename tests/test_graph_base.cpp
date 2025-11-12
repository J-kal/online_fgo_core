/**
 * @file test_graph_base.cpp
 * @brief Unit tests for GraphBase class
 * @author AI Assistant
 * @date 2025-10-24
 */

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include "online_fgo_core/graph/GraphBase.h"
#include "online_fgo_core/interface/ApplicationInterface.h"
#include "online_fgo_core/data/DataTypesFGO.h"
#include "online_fgo_core/data/DataTypesMeasurement.h"

using namespace fgo::graph;
using namespace fgo::core;
using namespace fgo::data;

/**
 * @brief Test fixture for GraphBase tests
 */
class GraphBaseTest : public ::testing::Test {
protected:
    std::shared_ptr<StandaloneApplication> app;
    
    void SetUp() override {
        app = std::make_shared<StandaloneApplication>("test_graph");
        // Note: ConsoleLogger doesn't have setLevel, so we accept default logging
    }
    
    void TearDown() override {
        app.reset();
    }
};

/**
 * @brief Mock concrete implementation of GraphBase for testing
 */
class MockGraph : public GraphBase {
public:
    explicit MockGraph(ApplicationInterface& app) : GraphBase(app) {}
    
    // Implement pure virtual methods
    StatusGraphConstruction constructFactorGraphOnIMU(
        std::vector<IMUMeasurement>& dataIMU) override {
        return StatusGraphConstruction::SUCCESSFUL;
    }
    
    StatusGraphConstruction constructFactorGraphOnTime(
        const std::vector<double>& stateTimestamps,
        std::vector<IMUMeasurement>& dataIMU) override {
        return StatusGraphConstruction::SUCCESSFUL;
    }
    
    double optimize(State& new_state) override {
        return 0.0;
    }
};

// ============================================================================
// Construction and Initialization Tests
// ============================================================================

TEST_F(GraphBaseTest, ConstructorInitialization) {
    ASSERT_NO_THROW({
        MockGraph graph(*app);
    });
}

TEST_F(GraphBaseTest, InitGraphWithValidState) {
    MockGraph graph(*app);
    
    // Create initial state
    State initState;
    initState.timestamp = TimeStamp(1000.0, 0);
    initState.state = gtsam::NavState(
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
        gtsam::Vector3(0, 0, 0)
    );
    initState.imuBias = gtsam::imuBias::ConstantBias();
    
    // Create pre-integrator params
    auto preIntParams = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0, 0, -9.81)
    );
    
    ASSERT_NO_THROW({
        graph.initGraph(initState, 1000.0, preIntParams);
    });
}

TEST_F(GraphBaseTest, InitSolverISAM2) {
    MockGraph graph(*app);
    
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    
    ASSERT_NO_THROW({
        graph.initSolver(params);
    });
}

TEST_F(GraphBaseTest, InitSolverBatch) {
    MockGraph graph(*app);
    
    gtsam::LevenbergMarquardtParams params;
    
    ASSERT_NO_THROW({
        graph.initSolver(params);
    });
}

// ============================================================================
// Graph Operations Tests
// ============================================================================

TEST_F(GraphBaseTest, ResetGraph) {
    MockGraph graph(*app);
    
    // The graph starts empty
    EXPECT_EQ(graph.size(), 0);
    
    // Add a dummy factor
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones());
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        gtsam::Symbol('x', 0),
        gtsam::Symbol('x', 1),
        gtsam::Pose3(),
        noise
    );
    
    EXPECT_GT(graph.size(), 0);
    
    // Reset should clear the graph
    graph.resetGraph();
    EXPECT_EQ(graph.size(), 0);
}

TEST_F(GraphBaseTest, UpdatePredictedBuffer) {
    MockGraph graph(*app);
    
    State state;
    state.timestamp = TimeStamp(1001.0, 0);
    state.state = gtsam::NavState(
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 2, 3)),
        gtsam::Vector3(0.1, 0.2, 0.3)
    );
    
    ASSERT_NO_THROW({
        graph.updatePredictedBuffer(state);
    });
}

// ============================================================================
// Parameter Access Tests
// ============================================================================

TEST_F(GraphBaseTest, GetParamPtr) {
    MockGraph graph(*app);
    
    auto paramPtr = graph.getParamPtr();
    ASSERT_NE(paramPtr, nullptr);
}

TEST_F(GraphBaseTest, GetSensorCalibManagerPtr) {
    MockGraph graph(*app);
    
    auto calibMgr = graph.getSensorCalibManagerPtr();
    ASSERT_NE(calibMgr, nullptr);
}

// ============================================================================
// Virtual Method Tests
// ============================================================================

TEST_F(GraphBaseTest, ConstructFactorGraphOnIMU) {
    MockGraph graph(*app);
    
    std::vector<IMUMeasurement> imuData;
    
    // Create sample IMU measurement
    IMUMeasurement imu;
    imu.timestamp = TimeStamp(1000.0, 0);
    imu.gyro = gtsam::Vector3(0.01, 0.02, 0.03);
    imu.accLin = gtsam::Vector3(0, 0, 9.81);
    imuData.push_back(imu);
    
    auto status = graph.constructFactorGraphOnIMU(imuData);
    EXPECT_EQ(status, StatusGraphConstruction::SUCCESSFUL);
}

TEST_F(GraphBaseTest, ConstructFactorGraphOnTime) {
    MockGraph graph(*app);
    
    std::vector<double> timestamps = {1000.0, 1000.1, 1000.2};
    std::vector<IMUMeasurement> imuData;
    
    auto status = graph.constructFactorGraphOnTime(timestamps, imuData);
    EXPECT_EQ(status, StatusGraphConstruction::SUCCESSFUL);
}

TEST_F(GraphBaseTest, OptimizeReturnsTime) {
    MockGraph graph(*app);
    
    State newState;
    newState.timestamp = TimeStamp(1000.0, 0);
    
    double optimizationTime = graph.optimize(newState);
    EXPECT_GE(optimizationTime, 0.0);
}

// ============================================================================
// Multi-threading Safety Tests
// ============================================================================

TEST_F(GraphBaseTest, ConcurrentUpdates) {
    MockGraph graph(*app);
    
    std::vector<std::thread> threads;
    std::atomic<int> successCount{0};
    
    // Spawn multiple threads updating the predicted buffer
    for (int i = 0; i < 10; ++i) {
        threads.emplace_back([&graph, &successCount, i]() {
            State state;
            state.timestamp = TimeStamp(1000.0 + i * 0.1, 0);
            state.state = gtsam::NavState(
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(i, i, i)),
                gtsam::Vector3(0, 0, 0)
            );
            
            try {
                graph.updatePredictedBuffer(state);
                successCount++;
            } catch (...) {
                // Failed
            }
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    EXPECT_EQ(successCount, 10);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
