/**
 * @file test_kimera_integration_interface.cpp
 * @brief Unit tests for KimeraIntegrationInterface
 * @author Kimera Integration Team
 * @date 2025-10-31
 */

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>

#include "online_fgo_core/integration/KimeraIntegrationInterface.h"
#include "online_fgo_core/graph/GraphTimeCentricKimera.h"
#include "online_fgo_core/interface/ApplicationInterface.h"
#include "online_fgo_core/data/DataTypesFGO.h"

using namespace fgo::integration;
using namespace fgo::graph;
using namespace fgo::core;
using namespace fgo::data;

/**
 * @brief Test fixture for KimeraIntegrationInterface tests
 */
class KimeraIntegrationInterfaceTest : public ::testing::Test {
protected:
    std::shared_ptr<StandaloneApplication> app_;
    std::unique_ptr<KimeraIntegrationInterface> interface_;
    IntegrationParameters params_;
    
    void SetUp() override {
        app_ = std::make_shared<StandaloneApplication>("test_kimera_interface");
        interface_ = std::make_unique<KimeraIntegrationInterface>(*app_);
        
        // Setup default parameters
        params_.use_imu = true;
        params_.use_gp_motion_prior = true;
        params_.optimize_on_add = false;
        params_.timestamp_tolerance = 0.01;  // 10ms
        
        // IMU parameters
        params_.imu_params.gyroscope_noise_density = 0.0003394;
        params_.imu_params.accelerometer_noise_density = 0.002;
        params_.imu_params.gyroscope_random_walk = 0.000038785;
        params_.imu_params.accelerometer_random_walk = 0.0003;
        params_.imu_params.integration_uncertainty = 0.0;
        params_.imu_params.gravity_magnitude = 9.81;
        
        // GP parameters
        params_.gp_params.qc_model = "WhiteNoise";
        params_.gp_params.sigma_position = 0.1;
        params_.gp_params.sigma_rotation = 0.05;
        
        // Optimization parameters
        params_.optimization_params.max_iterations = 100;
        params_.optimization_params.lambda_initial = 1e-5;
        params_.optimization_params.lambda_factor = 10.0;
        params_.optimization_params.relative_error_threshold = 1e-5;
        params_.optimization_params.absolute_error_threshold = 1e-5;
    }
    
    void TearDown() override {
        interface_.reset();
        app_.reset();
    }
    
    // Helper: Create IMU measurement
    Eigen::Vector3d createAccel(double ax = 0.0, double ay = 0.0, double az = 9.81) {
        return Eigen::Vector3d(ax, ay, az);
    }
    
    Eigen::Vector3d createGyro(double wx = 0.0, double wy = 0.0, double wz = 0.0) {
        return Eigen::Vector3d(wx, wy, wz);
    }
    
    // Helper: Create NavState at origin with zero velocity
    gtsam::NavState createNavState(const gtsam::Pose3& pose = gtsam::Pose3(),
                                    const gtsam::Vector3& velocity = gtsam::Vector3::Zero()) {
        return gtsam::NavState(pose, velocity);
    }
};

// =============================================================================
// INITIALIZATION TESTS
// =============================================================================

TEST_F(KimeraIntegrationInterfaceTest, InitializeSuccess) {
    ASSERT_TRUE(interface_->initialize(params_));
    EXPECT_TRUE(interface_->isInitialized());
}

TEST_F(KimeraIntegrationInterfaceTest, InitializeWithInvalidParams) {
    // Invalid params (negative tolerance)
    IntegrationParameters bad_params = params_;
    bad_params.timestamp_tolerance = -0.01;
    
    // Should still initialize (validation in implementation)
    EXPECT_TRUE(interface_->initialize(bad_params));
}

TEST_F(KimeraIntegrationInterfaceTest, DoubleInitialize) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    // Second initialization should succeed (reinitialize)
    EXPECT_TRUE(interface_->initialize(params_));
}

// =============================================================================
// STATE CREATION TESTS
// =============================================================================

TEST_F(KimeraIntegrationInterfaceTest, CreateSingleState) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    double timestamp = 1.0;
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias bias;
    
    auto handle = interface_->createStateAtTimestamp(timestamp, nav_state, bias);
    
    ASSERT_TRUE(handle.has_value());
    EXPECT_EQ(handle->timestamp, timestamp);
    EXPECT_GT(handle->state_index, 0);
}

TEST_F(KimeraIntegrationInterfaceTest, CreateMultipleStates) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    std::vector<double> timestamps = {1.0, 1.1, 1.2, 1.3, 1.4};
    std::vector<StateHandle> handles;
    
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias bias;
    
    for (double ts : timestamps) {
        auto handle = interface_->createStateAtTimestamp(ts, nav_state, bias);
        ASSERT_TRUE(handle.has_value());
        handles.push_back(handle.value());
    }
    
    // Verify all states created
    EXPECT_EQ(handles.size(), timestamps.size());
    
    // Verify state indices are sequential
    for (size_t i = 1; i < handles.size(); ++i) {
        EXPECT_GT(handles[i].state_index, handles[i-1].state_index);
    }
}

TEST_F(KimeraIntegrationInterfaceTest, CreateStateAtSameTimestamp) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    double timestamp = 1.0;
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias bias;
    
    // Create first state
    auto handle1 = interface_->createStateAtTimestamp(timestamp, nav_state, bias);
    ASSERT_TRUE(handle1.has_value());
    
    // Create second state at same timestamp (within tolerance)
    auto handle2 = interface_->createStateAtTimestamp(timestamp, nav_state, bias);
    ASSERT_TRUE(handle2.has_value());
    
    // Should return same state (within tolerance)
    EXPECT_EQ(handle1->state_index, handle2->state_index);
}

TEST_F(KimeraIntegrationInterfaceTest, CreateStatesOutOfOrder) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    // Create states out of chronological order
    std::vector<double> timestamps = {1.0, 1.3, 1.1, 1.4, 1.2};
    
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias bias;
    
    for (double ts : timestamps) {
        auto handle = interface_->createStateAtTimestamp(ts, nav_state, bias);
        ASSERT_TRUE(handle.has_value());
    }
    
    // Verify all states created successfully
    auto all_timestamps = interface_->getAllStateTimestamps();
    EXPECT_GE(all_timestamps.size(), timestamps.size());
}

// =============================================================================
// IMU DATA TESTS
// =============================================================================

TEST_F(KimeraIntegrationInterfaceTest, AddSingleIMUMeasurement) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    double timestamp = 1.0;
    Eigen::Vector3d accel = createAccel();
    Eigen::Vector3d gyro = createGyro();
    
    EXPECT_TRUE(interface_->addIMUData(timestamp, accel, gyro));
}

TEST_F(KimeraIntegrationInterfaceTest, AddMultipleIMUMeasurements) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    // Add IMU measurements at 200 Hz for 1 second
    double imu_rate = 200.0;  // Hz
    double duration = 1.0;     // seconds
    int num_measurements = static_cast<int>(imu_rate * duration);
    
    for (int i = 0; i < num_measurements; ++i) {
        double timestamp = i / imu_rate;
        Eigen::Vector3d accel = createAccel();
        Eigen::Vector3d gyro = createGyro();
        
        EXPECT_TRUE(interface_->addIMUData(timestamp, accel, gyro));
    }
}

TEST_F(KimeraIntegrationInterfaceTest, AddIMUDataBatch) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    std::vector<double> timestamps;
    std::vector<Eigen::Vector3d> accels;
    std::vector<Eigen::Vector3d> gyros;
    
    // Create 10 measurements
    for (int i = 0; i < 10; ++i) {
        timestamps.push_back(i * 0.01);
        accels.push_back(createAccel());
        gyros.push_back(createGyro());
    }
    
    size_t added = interface_->addIMUDataBatch(timestamps, accels, gyros);
    EXPECT_EQ(added, timestamps.size());
}

// =============================================================================
// OPTIMIZATION TESTS
// =============================================================================

TEST_F(KimeraIntegrationInterfaceTest, OptimizeWithNoStates) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    auto result = interface_->optimize();
    
    // Should fail or return empty result
    EXPECT_FALSE(result.success);
}

TEST_F(KimeraIntegrationInterfaceTest, OptimizeWithSingleState) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    // Create one state
    double timestamp = 1.0;
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias bias;
    
    auto handle = interface_->createStateAtTimestamp(timestamp, nav_state, bias);
    ASSERT_TRUE(handle.has_value());
    
    // Optimize (should succeed with single state)
    auto result = interface_->optimize();
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.num_optimized_states, 1);
}

TEST_F(KimeraIntegrationInterfaceTest, OptimizeWithMultipleStates) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    // Create states at 10 Hz for 1 second
    std::vector<double> timestamps;
    for (int i = 0; i < 10; ++i) {
        timestamps.push_back(i * 0.1);
    }
    
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias bias;
    
    // Create states
    for (double ts : timestamps) {
        auto handle = interface_->createStateAtTimestamp(ts, nav_state, bias);
        ASSERT_TRUE(handle.has_value());
    }
    
    // Add IMU data between states
    for (size_t i = 0; i < timestamps.size() - 1; ++i) {
        double t_start = timestamps[i];
        double t_end = timestamps[i + 1];
        double dt = (t_end - t_start) / 10.0;  // 10 measurements between states
        
        for (int j = 0; j < 10; ++j) {
            double t = t_start + j * dt;
            interface_->addIMUData(t, createAccel(), createGyro());
        }
    }
    
    // Optimize
    auto result = interface_->optimize();
    
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.num_optimized_states, timestamps.size());
    EXPECT_GT(result.num_iterations, 0);
}

TEST_F(KimeraIntegrationInterfaceTest, OptimizeAndGetLatestState) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    // Create states
    std::vector<double> timestamps = {1.0, 1.1, 1.2};
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias bias;
    
    for (double ts : timestamps) {
        interface_->createStateAtTimestamp(ts, nav_state, bias);
    }
    
    // Add IMU data
    for (size_t i = 0; i < timestamps.size() - 1; ++i) {
        double t_start = timestamps[i];
        double t_end = timestamps[i + 1];
        double dt = (t_end - t_start) / 10.0;
        
        for (int j = 0; j < 10; ++j) {
            interface_->addIMUData(t_start + j * dt, createAccel(), createGyro());
        }
    }
    
    // Optimize and get latest state
    gtsam::NavState latest_nav;
    gtsam::imuBias::ConstantBias latest_bias;
    
    EXPECT_TRUE(interface_->optimizeAndGetLatestState(latest_nav, latest_bias));
}

// =============================================================================
// RESULT RETRIEVAL TESTS
// =============================================================================

TEST_F(KimeraIntegrationInterfaceTest, GetOptimizedPose) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    double timestamp = 1.0;
    gtsam::Pose3 initial_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 2, 3));
    gtsam::NavState nav_state = createNavState(initial_pose);
    gtsam::imuBias::ConstantBias bias;
    
    auto handle = interface_->createStateAtTimestamp(timestamp, nav_state, bias);
    ASSERT_TRUE(handle.has_value());
    
    // Optimize
    interface_->optimize();
    
    // Get optimized pose
    auto opt_pose = interface_->getOptimizedPose(handle.value());
    ASSERT_TRUE(opt_pose.has_value());
    
    // Should be close to initial (no motion)
    EXPECT_TRUE(initial_pose.equals(opt_pose.value(), 0.1));
}

TEST_F(KimeraIntegrationInterfaceTest, GetOptimizedVelocity) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    double timestamp = 1.0;
    gtsam::Vector3 initial_velocity(1.0, 0.0, 0.0);
    gtsam::NavState nav_state = createNavState(gtsam::Pose3(), initial_velocity);
    gtsam::imuBias::ConstantBias bias;
    
    auto handle = interface_->createStateAtTimestamp(timestamp, nav_state, bias);
    ASSERT_TRUE(handle.has_value());
    
    // Optimize
    interface_->optimize();
    
    // Get optimized velocity
    auto opt_vel = interface_->getOptimizedVelocity(handle.value());
    ASSERT_TRUE(opt_vel.has_value());
}

TEST_F(KimeraIntegrationInterfaceTest, GetOptimizedBias) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    double timestamp = 1.0;
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias initial_bias(gtsam::Vector3(0.01, 0.02, 0.03),
                                                gtsam::Vector3(0.001, 0.002, 0.003));
    
    auto handle = interface_->createStateAtTimestamp(timestamp, nav_state, initial_bias);
    ASSERT_TRUE(handle.has_value());
    
    // Optimize
    interface_->optimize();
    
    // Get optimized bias
    auto opt_bias = interface_->getOptimizedBias(handle.value());
    ASSERT_TRUE(opt_bias.has_value());
}

TEST_F(KimeraIntegrationInterfaceTest, GetStateCovariance) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    double timestamp = 1.0;
    gtsam::NavState nav_state = createNavState();
    gtsam::imuBias::ConstantBias bias;
    
    auto handle = interface_->createStateAtTimestamp(timestamp, nav_state, bias);
    ASSERT_TRUE(handle.has_value());
    
    // Optimize
    interface_->optimize();
    
    // Get state covariance
    auto cov = interface_->getStateCovariance(handle.value());
    ASSERT_TRUE(cov.has_value());
    EXPECT_GT(cov->rows(), 0);
    EXPECT_GT(cov->cols(), 0);
}

// =============================================================================
// INTEGRATION TEST: Full Pipeline
// =============================================================================

TEST_F(KimeraIntegrationInterfaceTest, FullPipelineIntegration) {
    ASSERT_TRUE(interface_->initialize(params_));
    
    // Simulate 2 seconds of VIO at 10 Hz keyframes and 200 Hz IMU
    double keyframe_rate = 10.0;  // Hz
    double imu_rate = 200.0;      // Hz
    double duration = 2.0;        // seconds
    
    int num_keyframes = static_cast<int>(keyframe_rate * duration);
    
    std::vector<StateHandle> state_handles;
    
    // Create states at keyframe rate
    for (int i = 0; i < num_keyframes; ++i) {
        double timestamp = i / keyframe_rate;
        
        // Create state with some motion
        gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(timestamp, 0, 0));
        gtsam::Vector3 velocity(1.0, 0, 0);
        gtsam::NavState nav_state(pose, velocity);
        gtsam::imuBias::ConstantBias bias;
        
        auto handle = interface_->createStateAtTimestamp(timestamp, nav_state, bias);
        ASSERT_TRUE(handle.has_value());
        state_handles.push_back(handle.value());
        
        // Add IMU measurements until next keyframe
        if (i < num_keyframes - 1) {
            double t_start = timestamp;
            double t_end = (i + 1) / keyframe_rate;
            int num_imu = static_cast<int>((t_end - t_start) * imu_rate);
            
            for (int j = 0; j < num_imu; ++j) {
                double t_imu = t_start + j / imu_rate;
                interface_->addIMUData(t_imu, createAccel(), createGyro());
            }
        }
    }
    
    // Optimize
    auto result = interface_->optimize();
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.num_optimized_states, num_keyframes);
    EXPECT_GT(result.num_iterations, 0);
    EXPECT_LT(result.final_error, 1000.0);  // Reasonable error threshold
    
    // Verify we can retrieve results for all states
    for (const auto& handle : state_handles) {
        auto pose = interface_->getOptimizedPose(handle);
        EXPECT_TRUE(pose.has_value());
        
        auto vel = interface_->getOptimizedVelocity(handle);
        EXPECT_TRUE(vel.has_value());
        
        auto bias = interface_->getOptimizedBias(handle);
        EXPECT_TRUE(bias.has_value());
    }
}

// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
