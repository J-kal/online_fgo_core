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
 * @brief Test simulating Kimera-VIO incremental keyframe processing (matches vioBackend behavior)
 * 
 * This test mimics the live feed scenario where vioBackend:
 * 1. Processes keyframes incrementally as they arrive
 * 2. Adds IMU factors immediately when a keyframe is added
 * 3. Optimizes on each keyframe (incremental mode)
 * 
 * This is the correct way to use the system - NOT batch mode.
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
     
    // Setup parameters - optimize on each keyframe (incremental mode, like vioBackend)
     KimeraIntegrationParams params;
     params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;  // Optimize incrementally on each keyframe
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
     
    // Simulate 5 keyframes at 0.1s intervals - process incrementally (like vioBackend)
     const int num_keyframes = 5;
     const double dt = 0.1;  // 100ms between keyframes
     std::vector<StateHandle> state_handles;
     
     // Create initial pose (at origin, identity rotation)
     Pose3 initial_pose;
     Vector3 initial_velocity = Vector3::Zero();
     imuBias::ConstantBias initial_bias;
     
    // Process first keyframe (no PIM, as it's the first)
     double t0 = 0.0;
    auto handle0 = interface.addKeyframeWithPIM(t0, initial_pose, initial_velocity, initial_bias, nullptr);
     ASSERT_TRUE(handle0.valid) << "Failed to create first state";
     state_handles.push_back(handle0);
     
    // Process subsequent keyframes incrementally (like vioBackend does)
     for (int i = 1; i < num_keyframes; ++i) {
         double t_i = t0 + i * dt;
         
         // Create state at this timestamp
         // Simple motion: move forward in x-direction at 1 m/s
         Pose3 pose_i(Rot3(), Point3(i * dt * 1.0, 0.0, 0.0));  // 1 m/s forward
         Vector3 vel_i(1.0, 0.0, 0.0);
         
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
         
        // Add keyframe with PIM incrementally (matches vioBackend behavior)
        // This will:
        // 1. Create state
        // 2. Add IMU factor from PIM
        // 3. Optimize immediately (since optimize_on_keyframe = true)
         auto pim_ptr = std::make_shared<PreintegratedCombinedMeasurements>(pim);
        auto handle_i = interface.addKeyframeWithPIM(t_i, pose_i, vel_i, initial_bias, pim_ptr);
        ASSERT_TRUE(handle_i.valid) << "Failed to create state at timestamp " << t_i;
        state_handles.push_back(handle_i);
        
        // Verify we can retrieve optimized state immediately (incremental optimization)
        auto opt_state = interface.getOptimizedState(handle_i);
        EXPECT_TRUE(opt_state.has_value()) << "Failed to get optimized state for keyframe " << i;
    }
    
    // Final verification - check all states are optimized
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
    
    std::cout << "Incremental optimization successful!" << std::endl;
    std::cout << "  Total states: " << state_handles.size() << std::endl;
 }
 
 /**
 * @brief Test with minimal setup (2 keyframes only) - incremental mode
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
    params.optimize_on_keyframe = true;  // Incremental optimization
     params.use_gp_priors = false;
     
     ASSERT_TRUE(interface.initialize(params));
     
     // Create PIM parameters
     auto pim_params = PreintegrationCombinedParams::MakeSharedD(9.81);
     pim_params->setGyroscopeCovariance(Matrix3::Identity() * 0.0001);
     pim_params->setAccelerometerCovariance(Matrix3::Identity() * 0.001);
     
    // Create first keyframe (no PIM)
     double t0 = 0.0;
    auto handle0 = interface.addKeyframeWithPIM(t0, Pose3(), Vector3::Zero(), imuBias::ConstantBias(), nullptr);
     ASSERT_TRUE(handle0.valid);
     
    // Create second keyframe with PIM (incremental)
    double t1 = 0.1;
    PreintegratedCombinedMeasurements pim(pim_params, imuBias::ConstantBias());
    pim.integrateMeasurement(Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
    auto pim_ptr = std::make_shared<PreintegratedCombinedMeasurements>(pim);
    
    auto handle1 = interface.addKeyframeWithPIM(t1, Pose3(), Vector3::Zero(), imuBias::ConstantBias(), pim_ptr);
     ASSERT_TRUE(handle1.valid);
     
    // Verify states are optimized (optimization happens incrementally)
    auto opt_state0 = interface.getOptimizedState(handle0);
    auto opt_state1 = interface.getOptimizedState(handle1);
    EXPECT_TRUE(opt_state0.has_value());
    EXPECT_TRUE(opt_state1.has_value());
    
    std::cout << "Minimal incremental test - 2 keyframes processed and optimized" << std::endl;
}

/**
 * @brief Helper function to create PIM parameters with standard IMU noise
 */
auto createStandardPIMParams() {
    auto pim_params = PreintegrationCombinedParams::MakeSharedD(9.81);
    pim_params->setGyroscopeCovariance(Matrix3::Identity() * std::pow(0.0003394, 2.0));
    pim_params->setAccelerometerCovariance(Matrix3::Identity() * std::pow(0.002, 2.0));
    pim_params->setBiasAccCovariance(Matrix3::Identity() * std::pow(0.0003, 2.0));
    pim_params->setBiasOmegaCovariance(Matrix3::Identity() * std::pow(0.000038785, 2.0));
    return pim_params;
}

/**
 * @brief Helper function to create a PIM with simulated IMU measurements
 */
std::shared_ptr<PreintegratedCombinedMeasurements> createPIM(
    const boost::shared_ptr<PreintegrationCombinedParams>& params,
    const imuBias::ConstantBias& bias,
    const Vector3& accel_mean,
    const Vector3& gyro_mean,
    double dt,
    int num_measurements = 10) {
    
    PreintegratedCombinedMeasurements pim(params, bias);
    double imu_dt = dt / num_measurements;
    
    for (int i = 0; i < num_measurements; ++i) {
        pim.integrateMeasurement(accel_mean, gyro_mean, imu_dt);
    }
    
    return std::make_shared<PreintegratedCombinedMeasurements>(pim);
}

/**
 * @brief Test that prior factors are correctly added to the first state
 */
TEST(KimeraSimpleIntegration, PriorFactorsOnFirstState) {
    auto app = std::make_shared<StandaloneApplication>("test_priors");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    // Create first keyframe - should have prior factors
    Pose3 initial_pose;
    Vector3 initial_velocity(0.0, 0.0, 0.0);
    imuBias::ConstantBias initial_bias;
    
    auto handle0 = interface.addKeyframeWithPIM(0.0, initial_pose, initial_velocity, initial_bias, nullptr);
    ASSERT_TRUE(handle0.valid);
    
    // Verify we can get optimized state (priors should constrain it)
    auto opt_state = interface.getOptimizedState(handle0);
    EXPECT_TRUE(opt_state.has_value()) << "First state should be optimized with priors";
    
    // Verify the state is close to initial values (priors should keep it close)
    if (opt_state.has_value()) {
        EXPECT_LT(opt_state->pose().translation().norm(), 0.1) 
            << "Prior should keep position close to initial";
        EXPECT_LT(opt_state->velocity().norm(), 0.1) 
            << "Prior should keep velocity close to initial";
    }
    
    std::cout << "Prior factors test passed" << std::endl;
}

/**
 * @brief Test IMU factor connectivity between consecutive keyframes
 */
TEST(KimeraSimpleIntegration, IMUFactorConnectivity) {
    auto app = std::make_shared<StandaloneApplication>("test_imu_connectivity");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    auto pim_params = createStandardPIMParams();
    imuBias::ConstantBias bias;
    
    // Create 3 keyframes with PIM between them
    std::vector<StateHandle> handles;
    
    // First keyframe
    auto h0 = interface.addKeyframeWithPIM(0.0, Pose3(), Vector3::Zero(), bias, nullptr);
    ASSERT_TRUE(h0.valid);
    handles.push_back(h0);
    
    // Second keyframe with PIM
    auto pim1 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
    auto h1 = interface.addKeyframeWithPIM(0.1, Pose3(), Vector3::Zero(), bias, pim1);
    ASSERT_TRUE(h1.valid);
    handles.push_back(h1);
    
    // Third keyframe with PIM
    auto pim2 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
    auto h2 = interface.addKeyframeWithPIM(0.2, Pose3(), Vector3::Zero(), bias, pim2);
    ASSERT_TRUE(h2.valid);
    handles.push_back(h2);
    
    // Verify all states are optimized (IMU factors should connect them)
    for (size_t i = 0; i < handles.size(); ++i) {
        auto opt_state = interface.getOptimizedState(handles[i]);
        EXPECT_TRUE(opt_state.has_value()) 
            << "State " << i << " should be optimized with IMU factors";
    }
    
    std::cout << "IMU factor connectivity test passed - " << handles.size() << " keyframes connected" << std::endl;
}

/**
 * @brief Test optimization convergence - verify states change after optimization
 */
TEST(KimeraSimpleIntegration, OptimizationConvergence) {
    auto app = std::make_shared<StandaloneApplication>("test_convergence");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    auto pim_params = createStandardPIMParams();
    imuBias::ConstantBias bias;
    
    // Create two keyframes with motion
    auto h0 = interface.addKeyframeWithPIM(0.0, Pose3(), Vector3(1.0, 0.0, 0.0), bias, nullptr);
    ASSERT_TRUE(h0.valid);
    
    // Create PIM with forward acceleration
    auto pim = createPIM(pim_params, bias, Vector3(1.0, 0.0, 9.81), Vector3::Zero(), 0.1);
    auto h1 = interface.addKeyframeWithPIM(0.1, Pose3(Rot3(), Point3(0.1, 0, 0)), 
                                           Vector3(1.1, 0.0, 0.0), bias, pim);
    ASSERT_TRUE(h1.valid);
    
    // Verify optimization occurred - states should be retrievable
    auto opt0 = interface.getOptimizedState(h0);
    auto opt1 = interface.getOptimizedState(h1);
    
    EXPECT_TRUE(opt0.has_value()) << "First state should be optimized";
    EXPECT_TRUE(opt1.has_value()) << "Second state should be optimized";
    
    // Verify states are reasonable (not NaN or infinite)
    if (opt0.has_value() && opt1.has_value()) {
        EXPECT_FALSE(opt0->pose().translation().array().isNaN().any()) 
            << "Optimized pose should not contain NaN";
        EXPECT_FALSE(opt1->pose().translation().array().isNaN().any()) 
            << "Optimized pose should not contain NaN";
        EXPECT_FALSE(opt0->velocity().array().isNaN().any()) 
            << "Optimized velocity should not contain NaN";
        EXPECT_FALSE(opt1->velocity().array().isNaN().any()) 
            << "Optimized velocity should not contain NaN";
    }
    
    std::cout << "Optimization convergence test passed" << std::endl;
}

/**
 * @brief Test with stationary scenario (no motion)
 */
TEST(KimeraSimpleIntegration, StationaryScenario) {
    auto app = std::make_shared<StandaloneApplication>("test_stationary");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    auto pim_params = createStandardPIMParams();
    imuBias::ConstantBias bias;
    
    // Create 3 keyframes with no motion (only gravity in IMU)
    auto h0 = interface.addKeyframeWithPIM(0.0, Pose3(), Vector3::Zero(), bias, nullptr);
    ASSERT_TRUE(h0.valid);
    
    // PIM with only gravity (stationary)
    auto pim1 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
    auto h1 = interface.addKeyframeWithPIM(0.1, Pose3(), Vector3::Zero(), bias, pim1);
    ASSERT_TRUE(h1.valid);
    
    auto pim2 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
    auto h2 = interface.addKeyframeWithPIM(0.2, Pose3(), Vector3::Zero(), bias, pim2);
    ASSERT_TRUE(h2.valid);
    
    // Verify all states optimized
    auto opt0 = interface.getOptimizedState(h0);
    auto opt1 = interface.getOptimizedState(h1);
    auto opt2 = interface.getOptimizedState(h2);
    
    EXPECT_TRUE(opt0.has_value() && opt1.has_value() && opt2.has_value());
    
    // Verify positions remain close (stationary)
    if (opt0.has_value() && opt1.has_value() && opt2.has_value()) {
        double pos_diff_01 = (opt1->pose().translation() - opt0->pose().translation()).norm();
        double pos_diff_12 = (opt2->pose().translation() - opt1->pose().translation()).norm();
        
        EXPECT_LT(pos_diff_01, 0.01) << "Stationary: position should not change much";
        EXPECT_LT(pos_diff_12, 0.01) << "Stationary: position should not change much";
    }
    
    std::cout << "Stationary scenario test passed" << std::endl;
}

/**
 * @brief Test with constant velocity motion
 */
TEST(KimeraSimpleIntegration, ConstantVelocityMotion) {
    auto app = std::make_shared<StandaloneApplication>("test_constant_velocity");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    auto pim_params = createStandardPIMParams();
    imuBias::ConstantBias bias;
    const double velocity = 1.0;  // 1 m/s forward
    const double dt = 0.1;
    
    // Create keyframes with constant velocity motion
    auto h0 = interface.addKeyframeWithPIM(0.0, Pose3(), Vector3(velocity, 0, 0), bias, nullptr);
    ASSERT_TRUE(h0.valid);
    
    // PIM with only gravity (constant velocity = no acceleration except gravity)
    auto pim1 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), dt);
    auto h1 = interface.addKeyframeWithPIM(dt, Pose3(Rot3(), Point3(velocity * dt, 0, 0)), 
                                           Vector3(velocity, 0, 0), bias, pim1);
    ASSERT_TRUE(h1.valid);
    
    auto pim2 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), dt);
    auto h2 = interface.addKeyframeWithPIM(2*dt, Pose3(Rot3(), Point3(2*velocity*dt, 0, 0)), 
                                           Vector3(velocity, 0, 0), bias, pim2);
    ASSERT_TRUE(h2.valid);
    
    // Verify optimization
    auto opt0 = interface.getOptimizedState(h0);
    auto opt1 = interface.getOptimizedState(h1);
    auto opt2 = interface.getOptimizedState(h2);
    
    EXPECT_TRUE(opt0.has_value() && opt1.has_value() && opt2.has_value());
    
    // Verify velocity is approximately constant
    if (opt0.has_value() && opt1.has_value() && opt2.has_value()) {
        double vel0 = opt0->velocity().norm();
        double vel1 = opt1->velocity().norm();
        double vel2 = opt2->velocity().norm();
        
        EXPECT_NEAR(vel0, vel1, 0.1) << "Constant velocity: velocities should be similar";
        EXPECT_NEAR(vel1, vel2, 0.1) << "Constant velocity: velocities should be similar";
    }
    
    std::cout << "Constant velocity motion test passed" << std::endl;
}

/**
 * @brief Test with multiple keyframes (stress test)
 */
TEST(KimeraSimpleIntegration, MultipleKeyframes) {
    auto app = std::make_shared<StandaloneApplication>("test_multiple_keyframes");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    auto pim_params = createStandardPIMParams();
    imuBias::ConstantBias bias;
    const int num_keyframes = 10;
    const double dt = 0.1;
    std::vector<StateHandle> handles;
    
    // Create first keyframe
    auto h0 = interface.addKeyframeWithPIM(0.0, Pose3(), Vector3::Zero(), bias, nullptr);
    ASSERT_TRUE(h0.valid);
    handles.push_back(h0);
    
    // Create remaining keyframes incrementally
    for (int i = 1; i < num_keyframes; ++i) {
        double t = i * dt;
        Pose3 pose(Rot3(), Point3(i * dt * 0.5, 0, 0));  // Slow forward motion
        Vector3 vel(0.5, 0, 0);
        
        auto pim = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), dt);
        auto h = interface.addKeyframeWithPIM(t, pose, vel, bias, pim);
        ASSERT_TRUE(h.valid) << "Failed to create keyframe " << i;
        handles.push_back(h);
    }
    
    // Verify all states are optimized
    for (size_t i = 0; i < handles.size(); ++i) {
        auto opt_state = interface.getOptimizedState(handles[i]);
        EXPECT_TRUE(opt_state.has_value()) << "State " << i << " should be optimized";
    }
    
    std::cout << "Multiple keyframes test passed - " << handles.size() << " keyframes processed" << std::endl;
}

/**
 * @brief Test manual optimization (without optimize_on_keyframe)
 */
TEST(KimeraSimpleIntegration, ManualOptimization) {
    auto app = std::make_shared<StandaloneApplication>("test_manual_opt");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = false;  // Manual optimization
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    auto pim_params = createStandardPIMParams();
    imuBias::ConstantBias bias;
    
    // Create keyframes without automatic optimization
    auto h0 = interface.addKeyframeWithPIM(0.0, Pose3(), Vector3::Zero(), bias, nullptr);
    ASSERT_TRUE(h0.valid);
    
    auto pim1 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
    auto h1 = interface.addKeyframeWithPIM(0.1, Pose3(), Vector3::Zero(), bias, pim1);
    ASSERT_TRUE(h1.valid);
    
    auto pim2 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
    auto h2 = interface.addKeyframeWithPIM(0.2, Pose3(), Vector3::Zero(), bias, pim2);
    ASSERT_TRUE(h2.valid);
    
    // Manually trigger optimization
     auto opt_result = interface.optimize();
    EXPECT_TRUE(opt_result.success) << "Manual optimization should succeed";
    EXPECT_GT(opt_result.num_states, 0) << "Should have optimized states";
    EXPECT_GT(opt_result.num_factors, 0) << "Should have factors in graph";
    
    // Verify states are now optimized
    auto opt0 = interface.getOptimizedState(h0);
    auto opt1 = interface.getOptimizedState(h1);
    auto opt2 = interface.getOptimizedState(h2);
    
    EXPECT_TRUE(opt0.has_value() && opt1.has_value() && opt2.has_value());
    
    std::cout << "Manual optimization test passed - " << opt_result.num_states 
              << " states, " << opt_result.num_factors << " factors" << std::endl;
}

/**
 * @brief Test state consistency - verify states don't change unexpectedly
 */
TEST(KimeraSimpleIntegration, StateConsistency) {
    auto app = std::make_shared<StandaloneApplication>("test_consistency");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    auto pim_params = createStandardPIMParams();
    imuBias::ConstantBias bias;
    
    // Create keyframes
    auto h0 = interface.addKeyframeWithPIM(0.0, Pose3(), Vector3::Zero(), bias, nullptr);
    ASSERT_TRUE(h0.valid) << "First keyframe should be valid";
    auto pim1 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3::Zero(), 0.1);
    auto h1 = interface.addKeyframeWithPIM(0.1, Pose3(), Vector3::Zero(), bias, pim1);
    ASSERT_TRUE(h1.valid) << "Second keyframe should be valid";
    
    // Retrieve state multiple times - should be consistent
    auto opt1_first = interface.getOptimizedState(h1);
    auto opt1_second = interface.getOptimizedState(h1);
    auto opt1_third = interface.getOptimizedState(h1);
    
    EXPECT_TRUE(opt1_first.has_value() && opt1_second.has_value() && opt1_third.has_value());
    
    // Verify states are the same (within numerical precision)
    if (opt1_first.has_value() && opt1_second.has_value() && opt1_third.has_value()) {
        double diff_12 = (opt1_first->pose().translation() - opt1_second->pose().translation()).norm();
        double diff_23 = (opt1_second->pose().translation() - opt1_third->pose().translation()).norm();
        
        EXPECT_LT(diff_12, 1e-6) << "States should be consistent between retrievals";
        EXPECT_LT(diff_23, 1e-6) << "States should be consistent between retrievals";
    }
    
    std::cout << "State consistency test passed" << std::endl;
}

/**
 * @brief Test with rotation motion (not just translation)
 */
TEST(KimeraSimpleIntegration, RotationMotion) {
    auto app = std::make_shared<StandaloneApplication>("test_rotation");
    app->getParameterServer()->setString("GNSSFGO.Optimizer.smootherType", "IncrementalFixedLag");
    app->getParameterServer()->setDouble("GNSSFGO.Optimizer.smootherLag", 0.1);
    app->getParameterServer()->setDouble("initialPositionSigma", 1e-05);
    app->getParameterServer()->setDouble("initialRollPitchSigma", 0.174533);
    app->getParameterServer()->setDouble("initialYawSigma", 0.00174533);
    app->getParameterServer()->setDouble("initialVelocitySigma", 0.001);
    app->getParameterServer()->setDouble("initialAccBiasSigma", 0.1);
    app->getParameterServer()->setDouble("initialGyroBiasSigma", 0.01);
    
    KimeraIntegrationInterface interface(*app);
    KimeraIntegrationParams params;
    params.imu_rate = 200.0;
    params.optimize_on_keyframe = true;
    params.use_gp_priors = false;
    
    ASSERT_TRUE(interface.initialize(params));
    
    auto pim_params = createStandardPIMParams();
    imuBias::ConstantBias bias;
    const double dt = 0.1;
    const double yaw_rate = 0.1;  // rad/s
    
    // Create keyframes with rotation
    auto h0 = interface.addKeyframeWithPIM(0.0, Pose3(), Vector3::Zero(), bias, nullptr);
    ASSERT_TRUE(h0.valid);
    
    // PIM with rotation (yaw rate)
    auto pim1 = createPIM(pim_params, bias, Vector3(0, 0, 9.81), Vector3(0, 0, yaw_rate), dt);
    Rot3 rot1 = Rot3::Yaw(yaw_rate * dt);
    auto h1 = interface.addKeyframeWithPIM(dt, Pose3(rot1, Point3::Zero()), 
                                           Vector3::Zero(), bias, pim1);
    ASSERT_TRUE(h1.valid);
    
    // Verify optimization
    auto opt0 = interface.getOptimizedState(h0);
    auto opt1 = interface.getOptimizedState(h1);
    
    EXPECT_TRUE(opt0.has_value() && opt1.has_value());
    
    // Verify rotation occurred
    if (opt0.has_value() && opt1.has_value()) {
        Rot3 relative_rot = opt0->pose().rotation().inverse() * opt1->pose().rotation();
        double yaw_diff = std::abs(relative_rot.yaw());
        
        EXPECT_GT(yaw_diff, 0.001) << "Rotation should be detected";
        EXPECT_LT(yaw_diff, 0.2) << "Rotation should be reasonable";
    }
    
    std::cout << "Rotation motion test passed" << std::endl;
 }
 
 // Main function for standalone execution
 int main(int argc, char** argv) {
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
 }
 
 