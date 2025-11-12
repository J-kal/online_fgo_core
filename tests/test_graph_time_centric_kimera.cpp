/**
 * @file test_graph_time_centric_kimera.cpp
 * @brief Unit tests for GraphTimeCentricKimera
 * @author Kimera Integration Team
 * @date 2025-10-31
 */

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>

#include "online_fgo_core/graph/GraphTimeCentricKimera.h"
#include "online_fgo_core/interface/ApplicationInterface.h"
#include "online_fgo_core/data/DataTypesFGO.h"

using namespace fgo::graph;
using namespace fgo::core;
using namespace fgo::data;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

/**
 * @brief Test fixture for GraphTimeCentricKimera tests
 */
class GraphTimeCentricKimeraTest : public ::testing::Test {
protected:
    std::shared_ptr<StandaloneApplication> app_;
    std::unique_ptr<GraphTimeCentricKimera> graph_;
    
    void SetUp() override {
        app_ = std::make_shared<StandaloneApplication>("test_kimera_graph");
        graph_ = std::make_unique<GraphTimeCentricKimera>(*app_);
        
        // Initialize Kimera support
        ASSERT_TRUE(graph_->initializeKimeraSupport());
    }
    
    void TearDown() override {
        graph_.reset();
        app_.reset();
    }
    
    // Helper: Create IMU measurement
    IMUMeasurement createIMU(double timestamp, 
                             const gtsam::Vector3& accel = gtsam::Vector3(0, 0, 9.81),
                             const gtsam::Vector3& gyro = gtsam::Vector3::Zero()) {
        IMUMeasurement imu;
        imu.timestamp = TimeStamp(timestamp, 0);
        imu.accLin = accel;
        imu.gyro = gyro;
        imu.dt = 0.005;  // 5ms, 200 Hz
        imu.accLinCov = gtsam::Matrix33::Identity() * 0.01;
        imu.accRotCov = gtsam::Matrix33::Identity() * 0.01;
        return imu;
    }
};

// =============================================================================
// STATE CREATION TESTS
// =============================================================================

TEST_F(GraphTimeCentricKimeraTest, FindOrCreateSingleState) {
    double timestamp = 1.0;
    
    size_t state_idx = graph_->findOrCreateStateForTimestamp(timestamp, true);
    
    EXPECT_GT(state_idx, 0);
    EXPECT_TRUE(graph_->hasStateAtTimestamp(timestamp));
}

TEST_F(GraphTimeCentricKimeraTest, FindOrCreateMultipleStates) {
    std::vector<double> timestamps = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<size_t> state_indices;
    
    for (double ts : timestamps) {
        size_t idx = graph_->findOrCreateStateForTimestamp(ts, true);
        EXPECT_GT(idx, 0);
        state_indices.push_back(idx);
    }
    
    // Verify all states exist
    for (double ts : timestamps) {
        EXPECT_TRUE(graph_->hasStateAtTimestamp(ts));
    }
    
    // Verify indices are sequential
    for (size_t i = 1; i < state_indices.size(); ++i) {
        EXPECT_GT(state_indices[i], state_indices[i-1]);
    }
}

TEST_F(GraphTimeCentricKimeraTest, FindExistingStateWithinTolerance) {
    double timestamp = 1.0;
    double tolerance = 0.01;  // 10ms
    
    // Create first state
    size_t idx1 = graph_->findOrCreateStateForTimestamp(timestamp, true);
    EXPECT_GT(idx1, 0);
    
    // Try to create state within tolerance
    size_t idx2 = graph_->findOrCreateStateForTimestamp(timestamp + tolerance * 0.5, true);
    
    // Should return same state
    EXPECT_EQ(idx1, idx2);
}

TEST_F(GraphTimeCentricKimeraTest, CreateStateOutsideTolerance) {
    double timestamp1 = 1.0;
    double timestamp2 = 1.05;  // 50ms apart
    double tolerance = 0.01;   // 10ms
    
    // Create first state
    size_t idx1 = graph_->findOrCreateStateForTimestamp(timestamp1, true);
    
    // Create second state outside tolerance
    size_t idx2 = graph_->findOrCreateStateForTimestamp(timestamp2, true);
    
    // Should create new state
    EXPECT_NE(idx1, idx2);
    EXPECT_GT(idx2, idx1);
}

TEST_F(GraphTimeCentricKimeraTest, FindNonExistentStateWithoutCreate) {
    double timestamp = 1.0;
    
    size_t idx = graph_->findOrCreateStateForTimestamp(timestamp, false);
    
    // Should return 0 (not found)
    EXPECT_EQ(idx, 0);
}

TEST_F(GraphTimeCentricKimeraTest, GetAllStateTimestamps) {
    std::vector<double> timestamps = {1.0, 1.1, 1.2, 1.3};
    
    for (double ts : timestamps) {
        graph_->findOrCreateStateForTimestamp(ts, true);
    }
    
    auto retrieved = graph_->getAllStateTimestamps();
    
    EXPECT_EQ(retrieved.size(), timestamps.size());
    
    // Verify sorted
    for (size_t i = 1; i < retrieved.size(); ++i) {
        EXPECT_GT(retrieved[i], retrieved[i-1]);
    }
}

// =============================================================================
// IMU FACTOR TESTS
// =============================================================================

TEST_F(GraphTimeCentricKimeraTest, AddIMUMeasurements) {
    // Create IMU measurements
    std::vector<IMUMeasurement> imu_data;
    for (int i = 0; i < 20; ++i) {
        imu_data.push_back(createIMU(1.0 + i * 0.005));  // 200 Hz
    }
    
    EXPECT_TRUE(graph_->addIMUMeasurements(imu_data));
}

TEST_F(GraphTimeCentricKimeraTest, AddEmptyIMUMeasurements) {
    std::vector<IMUMeasurement> empty;
    
    EXPECT_FALSE(graph_->addIMUMeasurements(empty));
}

TEST_F(GraphTimeCentricKimeraTest, ConstructFactorGraphFromTimestamps) {
    // Create timestamps
    std::vector<double> timestamps = {1.0, 1.1, 1.2};
    
    // Add IMU data between timestamps
    for (size_t i = 0; i < timestamps.size() - 1; ++i) {
        double t_start = timestamps[i];
        double t_end = timestamps[i + 1];
        int num_imu = 20;  // 20 measurements between states
        
        for (int j = 0; j < num_imu; ++j) {
            double t = t_start + (t_end - t_start) * j / num_imu;
            graph_->addIMUMeasurements({createIMU(t)});
        }
    }
    
    // Construct factor graph
    auto status = graph_->constructFactorGraphFromTimestamps(timestamps);
    
    EXPECT_EQ(status, StatusGraphConstruction::SUCCESSFUL);
    
    // Verify states created
    for (double ts : timestamps) {
        EXPECT_TRUE(graph_->hasStateAtTimestamp(ts));
    }
}

TEST_F(GraphTimeCentricKimeraTest, ConstructFactorGraphWithNoTimestamps) {
    std::vector<double> empty;
    
    auto status = graph_->constructFactorGraphFromTimestamps(empty);
    
    EXPECT_EQ(status, StatusGraphConstruction::FAILED);
}

TEST_F(GraphTimeCentricKimeraTest, ConstructFactorGraphWithNoIMUData) {
    std::vector<double> timestamps = {1.0, 1.1, 1.2};
    
    // No IMU data added
    auto status = graph_->constructFactorGraphFromTimestamps(timestamps);
    
    // Should still succeed (creates states, warns about no IMU)
    EXPECT_NE(status, StatusGraphConstruction::FAILED);
}

// =============================================================================
// RESULT RETRIEVAL TESTS
// =============================================================================

TEST_F(GraphTimeCentricKimeraTest, GetOptimizedPose) {
    double timestamp = 1.0;
    
    // Create state
    size_t state_idx = graph_->findOrCreateStateForTimestamp(timestamp, true);
    ASSERT_GT(state_idx, 0);
    
    // Note: Without optimization, this may return initial value or nullopt
    auto pose = graph_->getOptimizedPose(state_idx, timestamp);
    
    // Just verify call doesn't crash
    // Actual pose value depends on optimization
}

TEST_F(GraphTimeCentricKimeraTest, GetOptimizedVelocity) {
    double timestamp = 1.0;
    
    size_t state_idx = graph_->findOrCreateStateForTimestamp(timestamp, true);
    ASSERT_GT(state_idx, 0);
    
    auto vel = graph_->getOptimizedVelocity(state_idx, timestamp);
    
    // Just verify call doesn't crash
}

TEST_F(GraphTimeCentricKimeraTest, GetOptimizedBias) {
    double timestamp = 1.0;
    
    size_t state_idx = graph_->findOrCreateStateForTimestamp(timestamp, true);
    ASSERT_GT(state_idx, 0);
    
    auto bias = graph_->getOptimizedBias(state_idx, timestamp);
    
    // Just verify call doesn't crash
}

TEST_F(GraphTimeCentricKimeraTest, GetStateCovariance) {
    double timestamp = 1.0;
    
    size_t state_idx = graph_->findOrCreateStateForTimestamp(timestamp, true);
    ASSERT_GT(state_idx, 0);
    
    auto cov = graph_->getStateCovariance(state_idx, timestamp);
    
    // Just verify call doesn't crash
}

// =============================================================================
// INTEGRATION TEST: Multiple States with IMU
// =============================================================================

TEST_F(GraphTimeCentricKimeraTest, IntegrationMultipleStatesWithIMU) {
    // Create 5 states at 10 Hz
    std::vector<double> timestamps;
    for (int i = 0; i < 5; ++i) {
        timestamps.push_back(1.0 + i * 0.1);
    }
    
    // Add IMU data at 200 Hz between states
    for (size_t i = 0; i < timestamps.size() - 1; ++i) {
        double t_start = timestamps[i];
        double t_end = timestamps[i + 1];
        double dt = (t_end - t_start) / 20.0;  // 20 measurements
        
        for (int j = 0; j < 20; ++j) {
            double t = t_start + j * dt;
            graph_->addIMUMeasurements({createIMU(t)});
        }
    }
    
    // Construct factor graph
    auto status = graph_->constructFactorGraphFromTimestamps(timestamps);
    EXPECT_EQ(status, StatusGraphConstruction::SUCCESSFUL);
    
    // Verify all states exist
    for (double ts : timestamps) {
        EXPECT_TRUE(graph_->hasStateAtTimestamp(ts));
        size_t idx = graph_->getStateIndexAtTimestamp(ts);
        EXPECT_GT(idx, 0);
    }
}

// =============================================================================
// EDGE CASE TESTS
// =============================================================================

TEST_F(GraphTimeCentricKimeraTest, CreateStatesInReverseOrder) {
    // Create states in reverse chronological order
    std::vector<double> timestamps = {1.4, 1.3, 1.2, 1.1, 1.0};
    
    for (double ts : timestamps) {
        size_t idx = graph_->findOrCreateStateForTimestamp(ts, true);
        EXPECT_GT(idx, 0);
    }
    
    // Verify all exist and can retrieve in sorted order
    auto sorted = graph_->getAllStateTimestamps();
    
    for (size_t i = 1; i < sorted.size(); ++i) {
        EXPECT_GT(sorted[i], sorted[i-1]);
    }
}

TEST_F(GraphTimeCentricKimeraTest, CreateStateDuplicateTimes) {
    double timestamp = 1.0;
    
    // Create same state multiple times
    size_t idx1 = graph_->findOrCreateStateForTimestamp(timestamp, true);
    size_t idx2 = graph_->findOrCreateStateForTimestamp(timestamp, true);
    size_t idx3 = graph_->findOrCreateStateForTimestamp(timestamp, true);
    
    // Should all return same index
    EXPECT_EQ(idx1, idx2);
    EXPECT_EQ(idx2, idx3);
}

TEST_F(GraphTimeCentricKimeraTest, VeryCloseTimestamps) {
    // Create states very close together (1 microsecond apart)
    double t1 = 1.000000;
    double t2 = 1.000001;
    
    size_t idx1 = graph_->findOrCreateStateForTimestamp(t1, true);
    size_t idx2 = graph_->findOrCreateStateForTimestamp(t2, true);
    
    // Depending on tolerance, may or may not be same state
    // Just verify no crash
    EXPECT_GT(idx1, 0);
    EXPECT_GT(idx2, 0);
}

// =============================================================================
// STRESS TEST
// =============================================================================

TEST_F(GraphTimeCentricKimeraTest, StressTestManyStates) {
    // Create 100 states over 10 seconds
    std::vector<double> timestamps;
    for (int i = 0; i < 100; ++i) {
        timestamps.push_back(1.0 + i * 0.1);
    }
    
    // Create all states
    for (double ts : timestamps) {
        size_t idx = graph_->findOrCreateStateForTimestamp(ts, true);
        EXPECT_GT(idx, 0);
    }
    
    // Verify all exist
    for (double ts : timestamps) {
        EXPECT_TRUE(graph_->hasStateAtTimestamp(ts));
    }
    
    // Verify correct count
    auto all_timestamps = graph_->getAllStateTimestamps();
    EXPECT_GE(all_timestamps.size(), timestamps.size());
}

TEST_F(GraphTimeCentricKimeraTest, StressTestManyIMUMeasurements) {
    // Add 10,000 IMU measurements (50 seconds at 200 Hz)
    std::vector<IMUMeasurement> imu_batch;
    
    for (int i = 0; i < 10000; ++i) {
        imu_batch.push_back(createIMU(1.0 + i * 0.005));
    }
    
    EXPECT_TRUE(graph_->addIMUMeasurements(imu_batch));
}

// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
