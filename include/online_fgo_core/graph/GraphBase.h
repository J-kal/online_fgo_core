//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
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

#ifndef ONLINE_FGO_CORE_GRAPHBASE_H
#define ONLINE_FGO_CORE_GRAPHBASE_H
#pragma once

#include <tuple>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

//boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

//gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

//internal - core
#include "online_fgo_core/interface/ApplicationInterface.h"
#include "online_fgo_core/interface/LoggerInterface.h"
#include "online_fgo_core/interface/ParameterInterface.h"
#include "online_fgo_core/interface/PublisherInterface.h"
#include "online_fgo_core/interface/TimeInterface.h"
#include "online_fgo_core/data/Buffer.h"
#include "online_fgo_core/graph/GraphUtils.h"
#include "online_fgo_core/graph/param/GraphParams.h"
#include "online_fgo_core/factor/FactorTypeID.h"
#include "online_fgo_core/factor/motion/ConstAngularRateFactor.h"
#include "online_fgo_core/factor/motion/ConstAccelerationFactor.h"
#include "online_fgo_core/factor/motion/ConstDriftFactor.h"
#include "online_fgo_core/factor/motion/ConstVelPriorFactor.h"
#include "online_fgo_core/factor/motion/GPWNOAPrior.h"
#include "online_fgo_core/factor/motion/GPWNOJPrior.h"
#include "online_fgo_core/factor/motion/GPWNOJPriorFull.h"
#include "online_fgo_core/factor/motion/GPSingerPrior.h"
#include "online_fgo_core/factor/motion/GPSingerPriorFull.h"
#include "online_fgo_core/solver/FixedLagSmoother.h"
#include "online_fgo_core/solver/BatchFixedLagSmoother.h"
#include "online_fgo_core/solver/IncrementalFixedLagSmoother.h"
#include "online_fgo_core/data/DataTypesFGO.h"
#include "online_fgo_core/data/Buffer.h"
#include "online_fgo_core/utils/NavigationTools.h"
#include "online_fgo_core/utils/Pose3Utils.h"
#include "online_fgo_core/utils/GPUtils.h"
#include "online_fgo_core/utils/AlgorithmicUtils.h"
#include "online_fgo_core/integrator/IntegratorBase.h"
#include "online_fgo_core/sensor/SensorCalibrationManager.h"

// Forward declarations to avoid circular dependencies
namespace fgo {
  namespace integrator {
    class IntegratorBase;  // Forward declare the class
    typedef std::map<std::string, std::shared_ptr<fgo::integrator::IntegratorBase>> IntegratorMap;
  }
}

namespace fgo::graph {
  using namespace fgo::utils;

  using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
  using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
  using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
  using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
  using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
  using gtsam::symbol_shorthand::N;  // integer ddambiguities
  using gtsam::symbol_shorthand::M;  // integer ambiguities for td cp without dd
  using gtsam::symbol_shorthand::A;  // acceleration
  using gtsam::symbol_shorthand::O;

  enum StateMeasSyncStatus {
    SYNCHRONIZED_I = 0,
    SYNCHRONIZED_J = 1,
    INTERPOLATED = 2,
    CACHED = 3,
    DROPPED = 4
  };

  enum StatusGraphConstruction {
    FAILED = 0,
    SUCCESSFUL = 1,
    NO_OPTIMIZATION = 2,
    PASSED = 3,
  };

  // Message types for residual publishing (framework-agnostic)
  struct FactorResidual {
    std::string factor_type;
    std::vector<double> residuals;
    double timestamp;
  };

  struct FactorResiduals {
    std::vector<FactorResidual> residuals;
    double timestamp;
  };

  class GraphBase : public gtsam::NonlinearFactorGraph, public std::enable_shared_from_this<GraphBase> {

  protected:
    std::string graphName_ = "OnlineFGO";
    fgo::core::ApplicationInterface* appPtr_;
    
    // Publishers for residuals (using generic publisher interface)
    std::shared_ptr<fgo::core::PublisherInterface<FactorResiduals>> pubResiduals_;
    std::shared_ptr<fgo::core::PublisherInterface<FactorResiduals>> pubGTResiduals_;

    std::shared_ptr<std::thread> pubResidualsThread_;
    std::atomic<bool> shouldPublishResiduals_{true};
    
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams_;
    GraphParamBasePtr graphBaseParamPtr_;
    std::shared_ptr<sensor::SensorCalibrationManager> sensorCalibManager_;

    fgo::integrator::IntegratorMap integratorMap_;
    std::shared_ptr<fgo::integrator::IntegratorBase> primarySensor_;

    fgo::solvers::FixedLagSmoother::KeyTimestampMap keyTimestampMap_;
    fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap currentKeyIndexTimestampMap_;
    gtsam::Values values_;
    uint64_t nState_ = 0; //counter for states
    std::atomic_bool isStateInited_{};
    std::unique_ptr<fgo::solvers::FixedLagSmoother> solver_;
    fgo::core::CircularDataBuffer<fgo::data::State> fgoOptStateBuffer_;
    std::vector<fgo::data::IMUMeasurement> dataIMURest_;

    fgo::core::CircularDataBuffer<std::pair<fgo::core::TimeStamp, double>> referenceSensorTimestampBuffer_;
    fgo::core::CircularDataBuffer<fgo::data::State> referenceStateBuffer_;
    fgo::core::CircularDataBuffer<gtsam::Vector6> accBuffer_;
    fgo::core::CircularDataBuffer<fgo::data::State> currentPredictedBuffer_;
    fgo::core::CircularDataBuffer<std::vector<gtsam::NonlinearFactor::shared_ptr>> factorBuffer_;
    fgo::core::CircularDataBuffer<std::pair<gtsam::Values, gtsam::Marginals>> resultMarginalBuffer_;
    gtsam::KeyVector relatedKeys_;

  protected: // protected functions

    inline void addAngularFactor(const gtsam::Key &omega_j, const gtsam::Key &bias_j,
                                 const fgo::data::IMUMeasurement &imuM,
                                 const gtsam::Vector3 &variance = gtsam::Vector3::Identity()) {
      gtsam::SharedNoiseModel noise_model;

      if (graphBaseParamPtr_->addEstimatedVarianceAfterInit)
        noise_model = gtsam::noiseModel::Diagonal::Variances(variance);
      else
        noise_model = gtsam::noiseModel::Diagonal::Sigmas(
          graphBaseParamPtr_->angularRateStd * gtsam::Vector3::Identity());

      this->emplace_shared<fgo::factor::ConstAngularRateFactor>(omega_j, bias_j, imuM.gyro, noise_model);
    }

    inline void addConstantAccelerationFactor(const gtsam::Key &accKey, const gtsam::Key &biasKey,
                                              const fgo::data::IMUMeasurement &imu,
                                              const boost::optional<gtsam::Matrix66> &accConv = boost::none) {
      gtsam::SharedNoiseModel noise_model;
      if (graphBaseParamPtr_->addEstimatedVarianceAfterInit && accConv)
        noise_model = gtsam::noiseModel::Gaussian::Covariance(*accConv);
      else
        noise_model = gtsam::noiseModel::Diagonal::Sigmas(
          graphBaseParamPtr_->angularRateStd *
          (gtsam::Vector6() << imu.accRotCov.diagonal(), imu.accLinCov.diagonal()).finished()
        );

      this->emplace_shared<fgo::factor::ConstAccelerationFactor>(accKey, biasKey,
                                                                 (gtsam::Vector6()
                                                                   << imu.accRot, imu.accLin).finished(),
                                                                 noise_model, graphBaseParamPtr_->AutoDiffNormalFactor);
    }

    inline void addConstDriftFactor(const gtsam::Key &cbd_i, const gtsam::Key &cbd_j, const double &dt,
                                    const gtsam::Vector2 &variance = gtsam::Vector2::Identity()) {
      gtsam::Vector2 var;
      if (graphBaseParamPtr_->addEstimatedVarianceAfterInit)
        var = variance;
      else
        var = gtsam::Vector2(std::pow(graphBaseParamPtr_->constBiasStd, 2),
                             std::pow(graphBaseParamPtr_->constDriftStd, 2));

      gtsam::SharedNoiseModel noise_model_cbd = assignNoiseModel(graphBaseParamPtr_->noiseModelClockFactor,
                                                                 var,
                                                                 graphBaseParamPtr_->robustParamClockFactor,
                                                                 "ConstClockDriftFactor",
                                                                 &appPtr_->getLogger());
      this->emplace_shared<fgo::factor::ConstDriftFactor>(cbd_i, cbd_j, dt, noise_model_cbd);
    }

    inline void addMotionModelFactor(const gtsam::Key &pose_i, const gtsam::Key &vel_i, const gtsam::Key &pose_j,
                                     const gtsam::Key &vel_j, const double &dt) {
      gtsam::SharedNoiseModel noise_model_mm =
        gtsam::noiseModel::Diagonal::Sigmas(graphBaseParamPtr_->motionModelStd * dt * gtsam::Vector6::Ones());
      this->emplace_shared<fgo::factor::MotionModelFactor>(pose_i, vel_i, pose_j, vel_j, dt, noise_model_mm);
    }

    inline void addGPMotionPrior(
      const gtsam::Key &poseKey_i, const gtsam::Key &velKey_i, const gtsam::Key &omegaKey_i, const gtsam::Key &accKey_i,
      const gtsam::Key &poseKey_j, const gtsam::Key &velKey_j, const gtsam::Key &omegaKey_j, const gtsam::Key &accKey_j,
      double dt,
      const boost::optional<gtsam::Vector6> &acc_i = boost::none,
      const boost::optional<gtsam::Vector6> &acc_j = boost::none,
      const boost::optional<gtsam::Matrix6> &ad = boost::none) {
      gtsam::SharedNoiseModel qcModel = gtsam::noiseModel::Diagonal::Variances(graphBaseParamPtr_->QcGPMotionPriorFull);

      if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOA) {
        this->emplace_shared<fgo::factor::GPWNOAPrior>(poseKey_i, velKey_i, omegaKey_i, poseKey_j, velKey_j,
                                                       omegaKey_j, dt, qcModel,
                                                       graphBaseParamPtr_->AutoDiffGPMotionPriorFactor,
                                                       graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOJ) {
        this->emplace_shared<fgo::factor::GPWNOJPrior>(poseKey_i, velKey_i, omegaKey_i, poseKey_j, velKey_j,
                                                       omegaKey_j, *acc_i, *acc_j, dt, qcModel,
                                                       graphBaseParamPtr_->AutoDiffGPMotionPriorFactor,
                                                       graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOJFull) {
        this->emplace_shared<fgo::factor::GPWNOJPriorFull>(poseKey_i, velKey_i, omegaKey_i, accKey_i,
                                                           poseKey_j, velKey_j, omegaKey_j, accKey_j,
                                                           dt, qcModel,
                                                           graphBaseParamPtr_->AutoDiffGPMotionPriorFactor,
                                                           graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian,
                                                           *acc_i, *acc_j);
      } else if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::Singer) {
        this->emplace_shared<fgo::factor::GPSingerPrior>(poseKey_i, velKey_i, omegaKey_i, poseKey_j, velKey_j,
                                                         omegaKey_j, dt, qcModel, *ad, *acc_i, *acc_j,
                                                         graphBaseParamPtr_->AutoDiffGPMotionPriorFactor,
                                                         graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::SingerFull) {
        this->emplace_shared<fgo::factor::GPSingerPriorFull>(poseKey_i, velKey_i, omegaKey_i, accKey_i,
                                                             poseKey_j, velKey_j, omegaKey_j, accKey_j,
                                                             dt, qcModel, *ad,
                                                             graphBaseParamPtr_->AutoDiffGPMotionPriorFactor,
                                                             graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian,
                                                             *acc_i, *acc_j);
      }
    }

    void notifyOptimization();

    void calculateResiduals(const fgo::core::TimeStamp &timestamp, const gtsam::Values &result,
                            const gtsam::Marginals &marginals);

    void calculateGroundTruthResiduals(const fgo::core::TimeStamp &timestamp, const gtsam::Values &gt);

  public:
    //constructor
    typedef std::shared_ptr<GraphBase> Ptr;
    typedef std::weak_ptr<GraphBase> WeakPtr;

    explicit GraphBase(fgo::core::ApplicationInterface& app);

    virtual ~GraphBase() {
      if(pubResidualsThread_ && pubResidualsThread_->joinable()) {
        shouldPublishResiduals_ = false;
        pubResidualsThread_->join();
      }
    }

    [[nodiscard]] GraphParamBasePtr getParamPtr() { return graphBaseParamPtr_; }

    [[nodiscard]] std::shared_ptr<sensor::SensorCalibrationManager> getSensorCalibManagerPtr() { return sensorCalibManager_; }

    /***
     * initialize the graph, used in the application after collecting all prior variables
     * @param initState state
     * @param initTimestamp timestamp, usually the timestamp of the reference sensor
     * @param preIntegratorParams param for imu pre-integration
     */
    virtual void initGraph(const fgo::data::State &initState,
                           const double &initTimestamp,
                           boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams);

    /***
     * init iSAM2
     * @param params iSAM2 params
     */
    void initSolver(const gtsam::ISAM2Params &params) {
      solver_ = std::make_unique<fgo::solvers::IncrementalFixedLagSmoother>(graphBaseParamPtr_->smootherLag, params);
    }

    /***
     * init batch
     * @param batch params
     */
    void initSolver(const gtsam::LevenbergMarquardtParams &params) {
      solver_ = std::make_unique<fgo::solvers::BatchFixedLagSmoother>(graphBaseParamPtr_->smootherLag, params);
    }

    /***
     * bridge the propagated state in the imu thread back for all sensors
     * @param state propagated state
     */
    void updatePredictedBuffer(const fgo::data::State &state) {
      currentPredictedBuffer_.update_buffer(state, state.timestamp);
    }

    /***
     * construct/extend the graph using imu as the timing reference
     * @param dataIMU
     * @return StatusGraphConstruction
     */
    virtual StatusGraphConstruction constructFactorGraphOnIMU(
      std::vector<fgo::data::IMUMeasurement> &dataIMU
    ) = 0;

    /***
     * construct/extend the graph using clock as the timing reference
     * @param stateTimestamps
     * @param dataIMU
     * @return StatusGraphConstruction
     */
    virtual StatusGraphConstruction constructFactorGraphOnTime(
      const std::vector<double> &stateTimestamps,
      std::vector<fgo::data::IMUMeasurement> &dataIMU
    ) = 0;

    /***
     * call the update function of solvers and collect state information
     * @param new_state
     * @return time for the optimization
     */
    virtual double optimize(fgo::data::State &new_state) = 0;

    /***
     * reset current temporary graph, the graph associated in the solver is not reset!
     */
    void resetGraph() {
      this->resize(0);
      keyTimestampMap_.clear();
      values_.clear();
      relatedKeys_.clear();
    }

    [[nodiscard]] const gtsam::Values &getValues() const { return values_; }

    [[nodiscard]] fgo::solvers::FixedLagSmoother::KeyTimestampMap &getNewKeyTimestampMap() { return keyTimestampMap_; }

    [[nodiscard]] const uint64_t &getStateIndex() const { return nState_; }

    [[nodiscard]] const std::vector<fgo::data::IMUMeasurement> &getRestIMUData() const { return dataIMURest_; }

    [[nodiscard]] std::vector<std::pair<fgo::core::TimeStamp, double>> getReferenceSensorMeasurementTime() {
      return referenceSensorTimestampBuffer_.get_all_buffer();
    };

    [[nodiscard]] std::vector<fgo::data::State> getReferenceStates() {
      return referenceStateBuffer_.get_all_buffer();
    };

    [[nodiscard]] std::vector<fgo::data::State> getReferenceStatesAndClean() {
      return referenceStateBuffer_.get_all_buffer_and_clean();
    };

    /**
    * Get an integrator pointer of base class that can be reinterpreted as the target integrator
    * @param name
    * @return
    */
    [[nodiscard]] std::shared_ptr<fgo::integrator::IntegratorBase> getIntegrator(const std::string &name) {
      auto iter = integratorMap_.find(name);
      if (iter != integratorMap_.end()) {
        return iter->second;
      } else
        return nullptr;
    }

    [[nodiscard]] bool isGraphInitialized() const { return isStateInited_; };

    /***
     * Update reference measurement timestamp
     * @param timestamp
     * @param rosTimestamp
     */
    void updateReferenceMeasurementTimestamp(const double &timestamp,
                                             const fgo::core::TimeStamp &rosTimestamp) {
      referenceSensorTimestampBuffer_.update_buffer(std::make_pair(rosTimestamp, timestamp), rosTimestamp);
    }

    /***
     * update the system prior states from a reference sensor
     * @param state
     * @param rosTimestamp
     */
    void updateReferenceState(const fgo::data::State &state,
                              const fgo::core::TimeStamp &rosTimestamp) {
      referenceStateBuffer_.update_buffer(state, rosTimestamp);
    }
  };
}

#endif //ONLINE_FGO_CORE_GRAPHBASE_H
