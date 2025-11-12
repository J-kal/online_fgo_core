//  Copyright 2023 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//  
//  ROS-agnostic port for online_fgo_core
//

#include "online_fgo_core/FGOLocalizationBase.h"
#include "online_fgo_core/utils/FGOParameter.h"
#include <chrono>
#include <stdexcept>

namespace fgo {

  FGOLocalizationBase::FGOLocalizationBase(fgo::core::ApplicationInterface* app)
    : appPtr_(app) {
    if (appPtr_ == nullptr) {
      throw std::invalid_argument("FGOLocalizationBase: ApplicationInterface pointer cannot be null");
    }
  }

  FGOLocalizationBase::~FGOLocalizationBase() {
    if (optThread_ && optThread_->joinable()) {
      optThread_->join();
    }
    if (initFGOThread_ && initFGOThread_->joinable()) {
      initFGOThread_->join();
    }
  }

  bool FGOLocalizationBase::initializeCommon() {
    auto& logger = appPtr_->getLogger();

    if (paramsPtr_ == nullptr) {
      paramsPtr_ = std::make_shared<GNSSFGOParams>();
    }

    // PARAM: General process logics
    fgo::utils::FGOParameter<std::string> sensor_param_prefix("GNSSFGO.VehicleParameterPrefix", "GNSSFGO", appPtr_);
    logger.warn("VehicleParameterPrefix: " + sensor_param_prefix.value());
    
    sensorCalibManager_ = std::make_shared<fgo::sensor::SensorCalibrationManager>(
      logger, appPtr_->getParameters(), sensor_param_prefix.value()
    );

    fgo::utils::FGOParameter<bool> offlineProcess("GNSSFGO.offlineProcess", false, appPtr_);
    paramsPtr_->offlineProcess = offlineProcess.value();
    logger.warn("offlineProcess: " + std::string(paramsPtr_->offlineProcess ? "true" : "false"));

    fgo::utils::FGOParameter<bool> cleanIMUonInit("GNSSFGO.cleanIMUonInit", true, appPtr_);
    paramsPtr_->cleanIMUonInit = cleanIMUonInit.value();
    logger.warn("cleanIMUOnInit: " + std::string(paramsPtr_->cleanIMUonInit ? "true" : "false"));

    fgo::utils::FGOParameter<bool> useHeaderTimestamp("GNSSFGO.useHeaderTimestamp", true, appPtr_);
    paramsPtr_->useHeaderTimestamp = useHeaderTimestamp.value();
    logger.info("useHeaderTimestamp: " + std::string(paramsPtr_->useHeaderTimestamp ? "true" : "false"));

    fgo::utils::FGOParameter<bool> delayFromPPS("GNSSFGO.delayFromPPS", false, appPtr_);
    paramsPtr_->delayFromPPS = delayFromPPS.value();
    logger.warn("delayFromPPS: " + std::string(paramsPtr_->delayFromPPS ? "true" : "false"));

    fgo::utils::FGOParameter<bool> verbose("GNSSFGO.verbose", false, appPtr_);
    paramsPtr_->verbose = verbose.value();
    logger.warn("verbose: " + std::string(paramsPtr_->verbose ? "true" : "false"));

    fgo::utils::FGOParameter<bool> useIMUAsTimeReference("GNSSFGO.useIMUAsTimeReference", false, appPtr_);
    paramsPtr_->useIMUAsTimeReference = useIMUAsTimeReference.value();
    logger.warn("useIMUAsTimeReference: " + std::string(paramsPtr_->useIMUAsTimeReference ? "true" : "false"));

    fgo::utils::FGOParameter<bool> calibGravity("GNSSFGO.calibGravity", true, appPtr_);
    paramsPtr_->calibGravity = calibGravity.value();
    logger.warn("calibGravity: " + std::string(paramsPtr_->calibGravity ? "true" : "false"));

    fgo::utils::FGOParameter<bool> calcErrorOnOpt("GNSSFGO.calcErrorOnOpt", false, appPtr_);
    paramsPtr_->calcErrorOnOpt = calcErrorOnOpt.value();
    logger.warn("calcErrorOnOpt: " + std::string(paramsPtr_->calcErrorOnOpt ? "true" : "false"));

    // PARAM: parameters
    fgo::utils::FGOParameter<int> optFrequency("GNSSFGO.optFrequency", 10, appPtr_);
    paramsPtr_->optFrequency = optFrequency.value();
    logger.warn("optFrequency: " + std::to_string(paramsPtr_->optFrequency));

    fgo::utils::FGOParameter<int> stateFrequency("GNSSFGO.stateFrequency", 10, appPtr_);
    paramsPtr_->stateFrequency = stateFrequency.value();
    logger.warn("stateFrequency: " + std::to_string(paramsPtr_->stateFrequency));

    fgo::utils::FGOParameter<double> pvtMeasTimeOffset("GNSSFGO.pvtMeasTimeOffset", 0.0, appPtr_);
    paramsPtr_->pvtMeasTimeOffset = pvtMeasTimeOffset.value();
    logger.warn("pvtMeasTimeOffset: " + std::to_string(paramsPtr_->pvtMeasTimeOffset));

    // PARAM: initialization variances
    fgo::utils::FGOParameter<std::vector<double>> initSigmaX(
      "GNSSFGO.Initialization.initSigmaX", 
      std::vector<double>{1.0, 1.0, 1.0, 0.1, 0.1, 0.1}, 
      appPtr_
    );
    lastOptimizedState_.poseVar = gtsam::Vector6(initSigmaX.value().data()).asDiagonal();
    logger.info("poseVar initialized");

    fgo::utils::FGOParameter<std::vector<double>> initSigmaV(
      "GNSSFGO.Initialization.initSigmaV", 
      std::vector<double>{0.1, 0.1, 0.1}, 
      appPtr_
    );
    lastOptimizedState_.velVar = gtsam::Vector3(initSigmaV.value().data()).asDiagonal();
    logger.info("velVar initialized");

    fgo::utils::FGOParameter<std::vector<double>> initSigmaW(
      "GNSSFGO.Initialization.initSigmaW", 
      std::vector<double>{0.01, 0.01, 0.01}, 
      appPtr_
    );
    lastOptimizedState_.omegaVar = gtsam::Vector3(initSigmaW.value().data()).asDiagonal();
    logger.info("omegaVar initialized");

    fgo::utils::FGOParameter<std::vector<double>> initSigmaB(
      "GNSSFGO.Initialization.initSigmaB", 
      std::vector<double>{1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4}, 
      appPtr_
    );
    lastOptimizedState_.imuBiasVar = gtsam::Vector6(initSigmaB.value().data()).asDiagonal();
    logger.info("imuBiasVar initialized");

    fgo::utils::FGOParameter<std::vector<double>> initSigmaC(
      "GNSSFGO.Initialization.initSigmaC", 
      std::vector<double>{100.0, 10.0}, 
      appPtr_
    );
    lastOptimizedState_.cbdVar = gtsam::Vector2(initSigmaC.value().data()).asDiagonal();
    logger.info("cbdVar initialized");

    fgo::utils::FGOParameter<double> accelerometerSigma("GNSSFGO.IMUPreintegrator.accelerometerSigma", 1.0, appPtr_);
    paramsPtr_->accelerometerSigma = accelerometerSigma.value();
    
    fgo::utils::FGOParameter<double> integrationSigma("GNSSFGO.IMUPreintegrator.integrationSigma", 1.0, appPtr_);
    paramsPtr_->integrationSigma = integrationSigma.value();
    logger.info("integrationSigma: " + std::to_string(paramsPtr_->integrationSigma));

    fgo::utils::FGOParameter<double> gyroscopeSigma("GNSSFGO.IMUPreintegrator.gyroscopeSigma", 1.0, appPtr_);
    paramsPtr_->gyroscopeSigma = gyroscopeSigma.value();
    logger.info("gyroscopeSigma: " + std::to_string(paramsPtr_->gyroscopeSigma));

    fgo::utils::FGOParameter<double> biasAccSigma("GNSSFGO.IMUPreintegrator.biasAccSigma", 4e-4, appPtr_);
    paramsPtr_->biasAccSigma = biasAccSigma.value();
    logger.info("biasAccSigma: " + std::to_string(paramsPtr_->biasAccSigma));

    fgo::utils::FGOParameter<double> biasOmegaSigma("GNSSFGO.IMUPreintegrator.biasOmegaSigma", 87e-5, appPtr_);
    paramsPtr_->biasOmegaSigma = biasOmegaSigma.value();
    logger.info("biasOmegaSigma: " + std::to_string(paramsPtr_->biasOmegaSigma));

    fgo::utils::FGOParameter<double> biasAccOmegaInt("GNSSFGO.IMUPreintegrator.biasAccOmegaInt", 87e-5, appPtr_);
    paramsPtr_->biasAccOmegaInt = biasAccOmegaInt.value();
    logger.info("biasAccOmegaInt: " + std::to_string(paramsPtr_->biasAccOmegaInt));

    fgo::utils::FGOParameter<int> IMUMeasurementFrequency("GNSSFGO.Graph.IMUMeasurementFrequency", 100, appPtr_);
    paramsPtr_->IMUMeasurementFrequency = IMUMeasurementFrequency.value();
    logger.info("IMUMeasurementFrequency: " + std::to_string(paramsPtr_->IMUMeasurementFrequency));

    fgo::utils::FGOParameter<int> bufferSize("GNSSFGO.bufferSize", 5, appPtr_);
    paramsPtr_->bufferSize = bufferSize.value();
    logger.warn("bufferSize: " + std::to_string(paramsPtr_->bufferSize));

    // PARAM: GP Prior
    fgo::utils::FGOParameter<std::string> gpType("GNSSFGO.Graph.gpType", "WNOA", appPtr_);
    fgo::utils::FGOParameter<bool> fullGP("GNSSFGO.Graph.fullGPPrior", false, appPtr_);
    logger.warn("gpType: " + gpType.value());
    
    if (gpType.value() == "WNOA") {
      paramsPtr_->gpType = fgo::data::GPModelType::WNOA;
    } else if (gpType.value() == "WNOJ") {
      paramsPtr_->gpType = fullGP.value() ? fgo::data::GPModelType::WNOJFull : fgo::data::GPModelType::WNOJ;
    } else if (gpType.value() == "Singer") {
      paramsPtr_->gpType = fullGP.value() ? fgo::data::GPModelType::SingerFull : fgo::data::GPModelType::Singer;
    } else {
      logger.warn("UNKNOWN GP motion prior " + gpType.value() + " given. Supported: GP-WNOA, GP-WNOJ, GP-Singer. Using GP-WNOA ...");
      paramsPtr_->gpType = fgo::data::GPModelType::WNOA;
    }

    fgo::utils::FGOParameter<bool> addGPPriorFactor("GNSSFGO.Graph.addGPPriorFactor", false, appPtr_);
    paramsPtr_->addGPPriorFactor = addGPPriorFactor.value();
    logger.info("addGPPriorFactor: " + std::string(paramsPtr_->addGPPriorFactor ? "true" : "false"));

    fgo::utils::FGOParameter<bool> addGPInterpolatedFactor("GNSSFGO.Graph.addGPInterpolatedFactor", false, appPtr_);
    paramsPtr_->addGPInterpolatedFactor = addGPInterpolatedFactor.value();
    logger.info("addGPInterpolatedFactor: " + std::string(paramsPtr_->addGPInterpolatedFactor ? "true" : "false"));

    // Create graph instance (pass reference, not pointer)
    graph_ = std::make_shared<fgo::graph::GraphTimeCentric>(*appPtr_);

    if (!paramsPtr_->offlineProcess) {
      logger.warn("GNSSFGO: starting ONLINE process by initializing the optimization and initFGO threads... ");

      // Start optimization thread
      optThread_ = std::make_unique<std::thread>([this]() -> void {
        if (paramsPtr_->useIMUAsTimeReference)
          this->timeCentricFGOonIMU();
        else
          this->timeCentricFGO();
      });

      // Start initialization thread
      initFGOThread_ = std::make_unique<std::thread>([this]() -> void {
        this->initializeFGO();
      });
    } else {
      logger.info("GNSSFGO: starting OFFLINE process... ");
    }

    // Resize buffers
    userEstimationBuffer_.resize_buffer(10);
    initGyroBiasBuffer_.resize_buffer(2);
    imuDataBuffer_.resize_buffer(1000 * paramsPtr_->bufferSize);
    fgoPredStateBuffer_.resize_buffer(50);
    fgoOptStateBuffer_.resize_buffer(50);
    referenceBuffer_.resize_buffer(100);

    return true;
  }

  bool FGOLocalizationBase::initializeFGO() {
    auto& logger = appPtr_->getLogger();
    logger.info("onInitializeFGO started on a different Thread... ");
    
    while (appPtr_->ok()) {
      std::unique_lock<std::mutex> lg(allBufferMutex_);
      conDoInit_.wait(lg, [&] { return triggeredInit_; });
      
      lastInitFinished_ = false;
      fgo::data::PVASolution foundPVA{};
      std::pair<fgo::core::TimeStamp, double> foundGNSSTime;

      isStateInited_ = false;

      const auto refSensorTimestamps = graph_->getReferenceSensorMeasurementTime();
      auto isPVAFound = false;

      if (refSensorTimestamps.empty() || !referenceBuffer_.size()) {
        lastInitFinished_ = true;
        triggeredInit_ = false;
        continue;
      }

      logger.info("Starting initializeFGO ...");
      const auto pvaBuffer = referenceBuffer_.get_all_buffer();
      auto refSensorTimestampIter = refSensorTimestamps.rbegin();
      
      while (refSensorTimestampIter != refSensorTimestamps.rend()) {
        auto pvaIter = pvaBuffer.rbegin();
        while (pvaIter != pvaBuffer.rend()) {
          if (pvaIter->tow == refSensorTimestampIter->second) {
            isPVAFound = true;
            logger.info("FGO Init.: found GNSS tow at: " + std::to_string(pvaIter->tow));
            foundPVA = *pvaIter;
            foundGNSSTime = *refSensorTimestampIter;
            break;
          }
          pvaIter++;
        }
        if (isPVAFound)
          break;
        refSensorTimestampIter++;
      }

      if (!isPVAFound) {
        logger.warn("FGO couldn't be initialized! Waiting ...");
        lastInitFinished_ = true;
        triggeredInit_ = false;
        continue;
      }

      if (isStateInited_) {
        logger.warn("is already initialized.");
      }

      const fgo::core::TimeStamp& initTime = foundGNSSTime.first;
      const auto this_imu = imuDataBuffer_.get_buffer(initTime);
      logger.info("(Re-)Initializing FGO current imu data size before cleaning: " + std::to_string(imuDataBuffer_.size()));

      imuDataBuffer_.cleanBeforeTime(fgo::core::toSeconds(initTime) - 0.02);

      lastOptimizedState_.timestamp = initTime;

      if (!paramsPtr_->initGyroBiasAsZero) {
        auto gyroBias = initGyroBiasBuffer_.get_last_buffer();
        lastOptimizedState_.imuBias = gtsam::imuBias::ConstantBias(
          (gtsam::Vector3() << 0., 0., 0.).finished(),
          (gtsam::Vector3() << gyroBias[0], gyroBias[1], gyroBias[2]).finished()
        );
      }
      
      lastOptimizedState_.cbd = (gtsam::Vector2() << foundPVA.clock_bias, foundPVA.clock_drift).finished();

      // Initialize state
      const auto reference_trans = sensorCalibManager_->getTransformationFromBase("reference");
      const auto init_imu_pos = foundPVA.xyz_ecef - foundPVA.rot_ecef.rotate(reference_trans.translation());
      const auto vecGrav = fgo::utils::gravity_ecef(init_imu_pos);
      const auto gravity_b = foundPVA.rot_ecef.unrotate(vecGrav);
      lastOptimizedState_.omega = this_imu.gyro;
      lastOptimizedState_.accMeasured = (gtsam::Vector6() << this_imu.accRot, this_imu.accLin + gravity_b).finished();

      const auto init_imu_vel = foundPVA.vel_ecef + foundPVA.rot_ecef.rotate(
        gtsam::skewSymmetric(reference_trans.translation()) * this_imu.gyro
      );
      lastOptimizedState_.state = gtsam::NavState(foundPVA.rot_ecef, init_imu_pos, init_imu_vel);

      // Setup IMU preintegrator
      preIntegratorParams_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(vecGrav);
      preIntegratorParams_->accelerometerCovariance = pow(paramsPtr_->accelerometerSigma, 2) * gtsam::I_3x3;
      preIntegratorParams_->integrationCovariance = pow(paramsPtr_->integrationSigma, 2) * gtsam::I_3x3;
      preIntegratorParams_->gyroscopeCovariance = pow(paramsPtr_->gyroscopeSigma, 2) * gtsam::I_3x3;
      preIntegratorParams_->biasAccCovariance = pow(paramsPtr_->biasAccSigma, 2) * gtsam::I_3x3;
      preIntegratorParams_->biasOmegaCovariance = pow(paramsPtr_->biasOmegaSigma, 2) * gtsam::I_3x3;
      preIntegratorParams_->biasAccOmegaInt = paramsPtr_->biasAccOmegaInt * gtsam::I_6x6;
      preIntegratorParams_->omegaCoriolis = gtsam::Vector3(0, 0, fgo::constants::earthRot);
      preIntegratorParams_->setUse2ndOrderCoriolis(true);
      
      currentIMUPreintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
        preIntegratorParams_, lastOptimizedState_.imuBias
      );

      lastOptimizedState_.poseVar = (gtsam::Vector6() << foundPVA.rot_var, foundPVA.xyz_var).finished().asDiagonal();

      graph_->initGraph(lastOptimizedState_, fgo::core::toSeconds(initTime), preIntegratorParams_);
      lastInitTimestamp_ = lastOptimizedState_.timestamp;

      currentPredState_ = lastOptimizedState_;
      const double processing_delay = (appPtr_->now() - initTime).toSec();
      fgoPredStateBuffer_.update_buffer(currentPredState_, initTime, processing_delay);
      graph_->updatePredictedBuffer(currentPredState_);
      fgoOptStateBuffer_.update_buffer(currentPredState_, initTime, processing_delay);
      
      // Publish initial state
      publishFGOState(lastOptimizedState_, "stateOptimized");
      publishPositionAsNavFix(lastOptimizedState_.state, lastOptimizedState_.timestamp, reference_trans.translation());

      logger.info("(Re-)Initializing FGO successful at: " + std::to_string(fgo::core::toSeconds(initTime)) +
                    " current IMU data size: " + std::to_string(imuDataBuffer_.size()));
      
      lastInitFinished_ = true;
      isStateInited_ = true;
      triggeredInit_ = false;
      logger.info("(Re-)Initializing FGO ...");
    }
    return true;
  }

  void FGOLocalizationBase::onIMUMsgCb(const fgo::data::IMUMeasurement& imuMeasurement) {
    static fgo::core::TimeStamp lastIMUTime{};
    static gtsam::Vector3 lastGyro{};
    static uint notifyCounter = paramsPtr_->IMUMeasurementFrequency / paramsPtr_->optFrequency;

    auto start_time = std::chrono::system_clock::now();

    const auto& ts = imuMeasurement.timestamp;

    // Update IMU buffer
    imuDataBuffer_.update_buffer(imuMeasurement, imuMeasurement.timestamp);

    if (!this->isStateInited_) {
      lastIMUTime = ts;
      lastGyro = imuMeasurement.gyro;
      if (lastInitFinished_) {
        triggeredInit_ = true;
        conDoInit_.notify_one();
      }
      return;
    }

    if (paramsPtr_->useIMUAsTimeReference) {
      if ((imuDataBuffer_.size() % notifyCounter) == 0) {
        if (lastOptFinished_) {
          appPtr_->getLogger().info("onIMU: Notify by IMU with imuBuffer: " + std::to_string(imuDataBuffer_.size()));
          this->notifyOptimization();
        } else {
          appPtr_->getLogger().error("onIMU: last optimization not finished! not triggering new optimization");
        }
      }
    }

    // Update predicted state
    const auto reference_trans = sensorCalibManager_->getTransformationFromBase("reference");

    gtsam::Vector3 gravity_b;
    if (paramsPtr_->calibGravity) {
      const auto gravity = fgo::utils::gravity_ecef(currentPredState_.state.position());
      gravity_b = currentPredState_.state.attitude().unrotate(gravity);
    }

    currentPredState_.mutex.lock();
    currentPredState_.timestamp = imuMeasurement.timestamp;
    currentPredState_.omega = currentPredState_.imuBias.correctGyroscope(imuMeasurement.gyro);
    currentPredState_.cbd = (gtsam::Matrix22() << 1, imuMeasurement.dt, 0, 1).finished() * currentPredState_.cbd;
    currentPredState_.accMeasured = (gtsam::Vector6() << 
      imuMeasurement.accRot, 
      currentPredState_.imuBias.correctAccelerometer(imuMeasurement.accLin + gravity_b)
    ).finished();

    if (isDoingPropagation_) {
      currentIMUPreIntMutex_.lock();
      currentIMUPreintegrator_->integrateMeasurement(
        imuMeasurement.accLin, imuMeasurement.gyro, imuMeasurement.dt
      );
      lastOptimizedState_.mutex.lock_shared();
      const auto new_state = currentIMUPreintegrator_->predict(
        lastOptimizedState_.state, lastOptimizedState_.imuBias
      );
      lastOptimizedState_.mutex.unlock_shared();
      currentIMUPreIntMutex_.unlock();
      currentPredState_.state = new_state;
    }
    currentPredState_.mutex.unlock();
    
    graph_->updatePredictedBuffer(currentPredState_);
    const double processing_delay = (appPtr_->now() - imuMeasurement.timestamp).toSec();
    fgoPredStateBuffer_.update_buffer(currentPredState_, imuMeasurement.timestamp, processing_delay);
    
    // Publish predicted state
    publishFGOState(currentPredState_, "statePredicted");

    if (this->isStateInited_ && !paramsPtr_->calcErrorOnOpt) {
      calculateErrorOnState(currentPredState_);
    }

    lastIMUTime = imuMeasurement.timestamp;
    lastGyro = imuMeasurement.gyro;

    double duration_cb = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::system_clock::now() - start_time
    ).count();
    
    if (duration_cb > 0.011) {
      appPtr_->getLogger().warn("onIMUMsgCb: cb takes " + std::to_string(duration_cb) + "s, more than expected 0.01s");
    }
  }

  void FGOLocalizationBase::timeCentricFGO() {
    auto& logger = appPtr_->getLogger();
    logger.info("Time centric graph optimization started in a different Thread... ");
    
    while (appPtr_->ok()) {
      static const double betweenOptimizationTime = 1. / paramsPtr_->optFrequency;
      static auto lastNotInitializedTimestamp = std::chrono::system_clock::now();
      static double lastGraphTimestamp = 0;
      static auto firstRun = true;

      if (!this->isStateInited_) {
        lastNotInitializedTimestamp = std::chrono::system_clock::now();
        firstRun = true;
        continue;
      }

      if (firstRun) {
        lastGraphTimestamp = fgo::core::toSeconds(lastInitTimestamp_);
        static uint notifyCounter = paramsPtr_->IMUMeasurementFrequency * betweenOptimizationTime;
        const auto imuSize = imuDataBuffer_.size();
        if (imuSize < notifyCounter) {
          continue;
        }
        firstRun = false;
      }

      const auto currentTime = appPtr_->now();
      double timeDiff = fgo::core::toSeconds(currentTime) - lastGraphTimestamp;

      if (timeDiff >= betweenOptimizationTime) {
        logger.info("onTimer: notify new optimization after: " + std::to_string(timeDiff) + " seconds");

        auto start = std::chrono::system_clock::now();

        std::vector<fgo::data::IMUMeasurement> imuData = imuDataBuffer_.get_all_buffer_and_clean();
        std::vector<double> newStateTimestamps;

        while (timeDiff >= betweenOptimizationTime) {
          lastGraphTimestamp += betweenOptimizationTime;
          logger.warn("Time-Centric Graph: create state at " + std::to_string(lastGraphTimestamp));
          newStateTimestamps.emplace_back(lastGraphTimestamp);
          timeDiff -= betweenOptimizationTime;
        }

        logger.warn("Time-Centric Graph: current IMU size " + std::to_string(imuData.size()));

        const auto constructGraphStatus = graph_->constructFactorGraphOnTime(newStateTimestamps, imuData);

        if (constructGraphStatus == fgo::graph::StatusGraphConstruction::FAILED) {
          throw std::invalid_argument("FGC failed");
        }

        double timeCFG = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::system_clock::now() - start
        ).count();

        if (constructGraphStatus == fgo::graph::StatusGraphConstruction::SUCCESSFUL) {
          logger.info("Start Optimization...");
          double timeOpt = this->optimize();
          isDoingPropagation_ = true;
          logger.info("Finished optimization with a duration: " + std::to_string(timeOpt));
          publishTiming(timeOpt);
        } else if (constructGraphStatus == fgo::graph::StatusGraphConstruction::NO_OPTIMIZATION) {
          isDoingPropagation_ = false;
        }
      }
    }
  }

  void FGOLocalizationBase::timeCentricFGOonIMU() {
    auto& logger = appPtr_->getLogger();
    logger.info("timeCentricFGOonIMU started on a different Thread... ");
    
    while (appPtr_->ok()) {
      logger.info("Starting optimization thread, waiting for trigger ...");
      std::unique_lock<std::mutex> lg(allBufferMutex_);
      conDoOpt_.wait(lg, [&] { return triggeredOpt_; });
      
      auto start = std::chrono::system_clock::now();
      std::vector<fgo::data::IMUMeasurement> imuData = imuDataBuffer_.get_all_buffer_and_clean();

      logger.warn("Triggered Optimization with " + std::to_string(imuData.size()) + " IMU data");

      const auto constructGraphStatus = graph_->constructFactorGraphOnIMU(imuData);

      if (constructGraphStatus == fgo::graph::StatusGraphConstruction::FAILED) {
        throw std::invalid_argument("FGC failed");
      }

      double timeCFG = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::system_clock::now() - start
      ).count();

      if (constructGraphStatus == fgo::graph::StatusGraphConstruction::SUCCESSFUL) {
        logger.info("Start Optimization...");
        double timeOpt = this->optimize();
        isDoingPropagation_ = true;
        logger.info("Finished optimization with a duration: " + std::to_string(timeOpt));
        publishTiming(timeOpt);
      } else if (constructGraphStatus == fgo::graph::StatusGraphConstruction::NO_OPTIMIZATION) {
        isDoingPropagation_ = false;
      }
      
      triggeredOpt_ = false;
      lastOptFinished_ = true;
    }
  }

  double FGOLocalizationBase::optimize() {
    const auto reference_trans = sensorCalibManager_->getTransformationFromBase("reference");
    fgo::data::State newOptState;
    double optTime = graph_->optimize(newOptState);
    
    fgoOptStateBuffer_.update_buffer(newOptState, newOptState.timestamp);
    publishFGOState(newOptState, "stateOptimized");
    publishPositionAsNavFix(newOptState.state, newOptState.timestamp, reference_trans.translation());

    lastOptimizedState_.mutex.lock();
    lastOptimizedState_ = newOptState;
    lastOptimizedState_.mutex.unlock();

    std::vector<fgo::data::IMUMeasurement> dataIMU = imuDataBuffer_.get_all_buffer();

    currentIMUPreIntMutex_.lock();
    preIntegratorParams_->n_gravity = fgo::utils::gravity_ecef(newOptState.state.position());
    currentIMUPreintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
      preIntegratorParams_, newOptState.imuBias
    );

    double dt = 0.0;
    for (const auto& meas_imu : dataIMU) {
      currentIMUPreintegrator_->integrateMeasurement(meas_imu.accLin, meas_imu.gyro, meas_imu.dt);
      dt += meas_imu.dt;
    }
    auto currentState = currentIMUPreintegrator_->predict(newOptState.state, newOptState.imuBias);
    currentIMUPreIntMutex_.unlock();

    currentPredState_.mutex.lock();
    currentPredState_.timestamp = fgo::core::addSeconds(newOptState.timestamp, dt);
    currentPredState_.state = currentState;
    
    if (paramsPtr_->addGPPriorFactor || paramsPtr_->addGPInterpolatedFactor) {
      if (!dataIMU.empty()) {
        currentPredState_.omega = newOptState.imuBias.correctGyroscope(dataIMU.back().gyro);
      } else {
        currentPredState_.omega = newOptState.omega;
      }
      currentPredState_.omegaVar = newOptState.omegaVar;
    }
    
    currentPredState_.cbd = newOptState.cbd;
    currentPredState_.cbdVar = newOptState.cbdVar;
    currentPredState_.imuBias = newOptState.imuBias;
    currentPredState_.imuBiasVar = newOptState.imuBiasVar;
    currentPredState_.poseVar = newOptState.poseVar;
    currentPredState_.velVar = newOptState.velVar;
    currentPredState_.ddIntAmb = newOptState.ddIntAmb;
    currentPredState_.ddIntAmbVar = newOptState.ddIntAmbVar;
    currentPredState_.mutex.unlock();

    if (this->isStateInited_ && paramsPtr_->calcErrorOnOpt) {
      calculateErrorOnState(newOptState);
    }

    return optTime;
  }

  void FGOLocalizationBase::calculateErrorOnState(const fgo::data::State& stateIn) {
    // This is a placeholder - derived classes can override with actual error calculation
    // For now, just log that error calculation was called
    appPtr_->getLogger().debug("calculateErrorOnState called for timestamp: " + 
                                  std::to_string(fgo::core::toSeconds(stateIn.timestamp)));
  }

  // Default publishing implementations (can be overridden by derived classes)
  void FGOLocalizationBase::publishFGOState(const fgo::data::State& state, const std::string& topic) {
    // Default implementation - derived classes should override for framework-specific publishing
    appPtr_->getLogger().debug("publishFGOState called for topic: " + topic);
  }

  void FGOLocalizationBase::publishPositionAsNavFix(
    const gtsam::NavState& state,
    const fgo::core::TimeStamp& timestamp,
    const gtsam::Point3& leverArm) {
    // Default implementation - derived classes should override
    appPtr_->getLogger().debug("publishPositionAsNavFix called");
  }

  void FGOLocalizationBase::publishPose(const fgo::data::State& state) {
    // Default implementation - derived classes should override
    appPtr_->getLogger().debug("publishPose called");
  }

  void FGOLocalizationBase::publishVelocity(const fgo::data::State& state) {
    // Default implementation - derived classes should override
    appPtr_->getLogger().debug("publishVelocity called");
  }

  void FGOLocalizationBase::publishTiming(double elapsedTime) {
    // Default implementation - derived classes should override
    appPtr_->getLogger().debug("publishTiming called with time: " + std::to_string(elapsedTime));
  }

  void FGOLocalizationBase::publishError(const fgo::data::State& state) {
    // Default implementation - derived classes should override
    appPtr_->getLogger().debug("publishError called");
  }

} // namespace fgo
