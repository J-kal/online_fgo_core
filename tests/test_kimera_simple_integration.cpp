/**
 * @file test_kimera_simple_integration.cpp
 * @brief Simple test simulating Kimera-VIO input to verify factor graph construction and optimization
 * @author Kimera Integration Team
 */

 #include <gtest/gtest.h>
 #include <gtsam/inference/Symbol.h>
 #include <gtsam/navigation/ImuBias.h>
 #include <gtsam/navigation/NavState.h>
 #include <gtsam/geometry/Pose3.h>
 // PreintegratedCombinedMeasurements is defined in CombinedImuFactor.h
 #include <gtsam/navigation/CombinedImuFactor.h>
 
 #include "online_fgo_core/integration/KimeraIntegrationInterface.h"
 #include "online_fgo_core/interface/ApplicationInterface.h"
 #include "online_fgo_core/data/DataTypesFGO.h"
 
 using namespace fgo::integration;
 using namespace fgo::core;
 using namespace gtsam;
 
 /**
  * @brief Simple test simulating Kimera-VIO keyframe input
  * 
  * This test:
  * 1. Creates 5 keyframes at 0.1s intervals
  * 2. Creates PIM data between consecutive keyframes
  * 3. Adds states and PIM data to the graph
  * 4. Optimizes the graph
  * 5. Verifies optimization succeeds
  */
 TEST(KimeraSimpleIntegration, BuildAndOptimizeGraph) {
     // Create application interface
     auto app = std::make_shared<StandaloneApplication>("test_kimera_simple");
     
    // Configure required parameters for GraphBase initialization
    // smootherType is required to initialize the solver (BatchFixedLag or IncrementalFixedLag)
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    
    // Set initialization parameters matching BackendParams.yaml (matching Kimera-VIO defaults)
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);  // 10.0/180.0*M_PI
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);      // 0.1/180.0*M_PI
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
     
     // Create integration interface
     KimeraIntegrationInterface interface(*app);
     
     // Setup parameters
     KimeraIntegrationParams params;
     params.imu_rate = 200.0;
     params.optimize_on_keyframe = false;  // We'll optimize manually
     params.use_gp_priors = false;  // Disabled for IMU isolation
     
     // Initialize
     ASSERT_TRUE(interface.initialize(params)) << "Failed to initialize interface";
     EXPECT_TRUE(interface.isInitialized());
     
     // Create PIM parameters (matching typical IMU specs)
     auto pim_params = PreintegrationCombinedParams::MakeSharedD(9.81);
     pim_params->setGyroscopeCovariance(Matrix3::Identity() * std::pow(0.0003394, 2.0));
     pim_params->setAccelerometerCovariance(Matrix3::Identity() * std::pow(0.002, 2.0));
     pim_params->setBiasAccCovariance(Matrix3::Identity() * std::pow(0.0003, 2.0));
     pim_params->setBiasOmegaCovariance(Matrix3::Identity() * std::pow(0.000038785, 2.0));
     
     // Simulate 5 keyframes at 0.1s intervals
     const int num_keyframes = 5;
     const double dt = 0.1;  // 100ms between keyframes
     std::vector<double> timestamps;
     std::vector<StateHandle> state_handles;
     std::vector<std::pair<double, std::shared_ptr<gtsam::PreintegrationType>>> pim_data;
     
     // Create initial pose (at origin, identity rotation)
     Pose3 initial_pose;
     Vector3 initial_velocity = Vector3::Zero();
     imuBias::ConstantBias initial_bias;
     
     // Create first keyframe state
     double t0 = 0.0;
     timestamps.push_back(t0);
     auto handle0 = interface.createStateAtTimestamp(t0, initial_pose, initial_velocity, initial_bias);
     ASSERT_TRUE(handle0.valid) << "Failed to create first state";
     state_handles.push_back(handle0);
     
     // Create subsequent keyframes and PIM data
     for (int i = 1; i < num_keyframes; ++i) {
         double t_i = t0 + i * dt;
         timestamps.push_back(t_i);
         
         // Create state at this timestamp
         // Simple motion: move forward in x-direction at 1 m/s
         Pose3 pose_i(Rot3(), Point3(i * dt * 1.0, 0.0, 0.0));  // 1 m/s forward
         Vector3 vel_i(1.0, 0.0, 0.0);
         
         auto handle_i = interface.createStateAtTimestamp(t_i, pose_i, vel_i, initial_bias);
         ASSERT_TRUE(handle_i.valid) << "Failed to create state at timestamp " << t_i;
         state_handles.push_back(handle_i);
         
         // Create PIM from previous keyframe to this one
         // Simulate constant acceleration forward (small, realistic)
         PreintegratedCombinedMeasurements pim(pim_params, initial_bias);
         
         // Integrate a few IMU measurements between keyframes
         // Simulate 10 IMU measurements per keyframe interval (10ms each)
         const int num_imu_per_interval = 10;
         const double imu_dt = dt / num_imu_per_interval;
         
         for (int j = 0; j < num_imu_per_interval; ++j) {
             // Simulate IMU measurements
             // Accelerometer: gravity + small forward acceleration
             Vector3 acc_measured(0.1, 0.0, 9.81);  // Small forward accel + gravity
             // Gyroscope: small rotation (simulating slight turn)
             Vector3 gyro_measured(0.0, 0.0, 0.01);  // Small yaw rate
             
             pim.integrateMeasurement(acc_measured, gyro_measured, imu_dt);
         }
         
         // Store PIM with destination timestamp
         auto pim_ptr = std::make_shared<PreintegratedCombinedMeasurements>(pim);
         pim_data.push_back(std::make_pair(t_i, pim_ptr));
     }
     
     // Add PIM data to interface
     EXPECT_TRUE(interface.addPreintegratedIMUData(pim_data)) 
         << "Failed to add preintegrated IMU data";
     
     // Optimize the graph
     auto opt_result = interface.optimize();
     
     // Verify optimization succeeded
     EXPECT_TRUE(opt_result.success) << "Optimization failed: " << opt_result.error_message;
     EXPECT_GT(opt_result.num_states, 0) << "No states optimized";
     EXPECT_GT(opt_result.num_factors, 0) << "No factors in graph";
     
     std::cout << "Optimization successful!" << std::endl;
     std::cout << "  States: " << opt_result.num_states << std::endl;
     std::cout << "  Factors: " << opt_result.num_factors << std::endl;
     std::cout << "  Optimization time: " << opt_result.optimization_time_ms << " ms" << std::endl;
     
     // Verify we can retrieve optimized states
     for (size_t i = 0; i < state_handles.size(); ++i) {
         auto opt_state = interface.getOptimizedState(state_handles[i]);
         EXPECT_TRUE(opt_state.has_value()) << "Failed to get optimized state for state " << i;
         
         if (opt_state.has_value()) {
             std::cout << "State " << i << " optimized pose: " 
                       << opt_state->pose().translation().transpose() << std::endl;
             std::cout << "State " << i << " optimized velocity: " 
                       << opt_state->velocity().transpose() << std::endl;
         }
     }
 }
 
 /**
  * @brief Test with minimal setup (2 keyframes only)
  */
 TEST(KimeraSimpleIntegration, MinimalTwoKeyframes) {
     auto app = std::make_shared<StandaloneApplication>("test_kimera_minimal");
     
    // Configure required parameters for GraphBase initialization
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    
    // Set initialization parameters matching BackendParams.yaml (matching Kimera-VIO defaults)
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);  // 10.0/180.0*M_PI
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);      // 0.1/180.0*M_PI
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
     
     KimeraIntegrationInterface interface(*app);
     
     KimeraIntegrationParams params;
     params.imu_rate = 200.0;
     params.optimize_on_keyframe = false;
     params.use_gp_priors = false;
     
     ASSERT_TRUE(interface.initialize(params));
     
     // Create PIM parameters
     auto pim_params = PreintegrationCombinedParams::MakeSharedD(9.81);
     pim_params->setGyroscopeCovariance(Matrix3::Identity() * 0.0001);
     pim_params->setAccelerometerCovariance(Matrix3::Identity() * 0.001);
     
     // Create two states
     double t0 = 0.0;
     double t1 = 0.1;
     
     auto handle0 = interface.createStateAtTimestamp(t0, Pose3(), Vector3::Zero(), imuBias::ConstantBias());
     ASSERT_TRUE(handle0.valid);
     
     auto handle1 = interface.createStateAtTimestamp(t1, Pose3(), Vector3::Zero(), imuBias::ConstantBias());
     ASSERT_TRUE(handle1.valid);
     
     // Create simple PIM
     PreintegratedCombinedMeasurements pim(pim_params, imuBias::ConstantBias());
     pim.integrateMeasurement(Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
     
     std::vector<std::pair<double, std::shared_ptr<gtsam::PreintegrationType>>> pim_data;
     pim_data.push_back(std::make_pair(t1, std::make_shared<PreintegratedCombinedMeasurements>(pim)));
     
     EXPECT_TRUE(interface.addPreintegratedIMUData(pim_data));
     
     // Optimize
     auto opt_result = interface.optimize();
     EXPECT_TRUE(opt_result.success) << "Optimization failed: " << opt_result.error_message;
     
     std::cout << "Minimal test - States: " << opt_result.num_states 
               << ", Factors: " << opt_result.num_factors 
               << ", Time: " << opt_result.optimization_time_ms << " ms" << std::endl;
 }
 
 // Main function for standalone execution
 int main(int argc, char** argv) {
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
 }
 
 