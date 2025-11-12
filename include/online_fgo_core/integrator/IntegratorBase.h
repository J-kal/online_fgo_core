//  Copyright 2021 Institute of Automatic Control RWTH Aachen University
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

#ifndef ONLINE_FGO_CORE_INTEGRATIONBASE_H
#define ONLINE_FGO_CORE_INTEGRATIONBASE_H
#pragma once

#include <tuple>
#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>
#include <gtsam/navigation/AttitudeFactor.h>

#include "online_fgo_core/graph/GraphBase.h"
#include "online_fgo_core/data/DataTypesFGO.h"
#include "online_fgo_core/factor/motion/GPWNOAPrior.h"
#include "online_fgo_core/factor/motion/GPWNOJPrior.h"
#include "online_fgo_core/model/gp_interpolator/GPWNOAInterpolator.h"
#include "online_fgo_core/model/gp_interpolator/GPWNOJInterpolator.h"
#include "online_fgo_core/model/gp_interpolator/GPWNOJInterpolatorFull.h"
#include "online_fgo_core/model/gp_interpolator/GPSingerInterpolator.h"
#include "online_fgo_core/model/gp_interpolator/GPSingerInterpolatorFull.h"
// #include "online_fgo_core/utils/MeasurmentDelayCalculator.h"  // File doesn't exist, not used
#include "online_fgo_core/graph/GraphUtils.h"
#include "online_fgo_core/integrator/param/IntegratorParams.h"
#include "online_fgo_core/factor/odometry/NavAttitudeFactor.h"
#include "online_fgo_core/factor/odometry/GPInterpolatedNavAttitudeFactor.h"
#include "online_fgo_core/factor/odometry/NavVelocityFactor.h"
#include "online_fgo_core/factor/odometry/GPInterpolatedNavVelocityFactor.h"
#include "online_fgo_core/factor/odometry/NavPoseFactor.h"
#include "online_fgo_core/factor/odometry/GPInterpolatedNavPoseFactor.h"
#include "online_fgo_core/solver/FixedLagSmoother.h"
// Forward declaration to avoid ROS dependency
namespace fgo::sensor {
  class SensorCalibrationManager;
}

namespace fgo::graph {
  class GraphBase;
}

namespace fgo::integrator {
  using namespace fgo::integrator::param;
  using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
  using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
  using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
  using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
  using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
  using gtsam::symbol_shorthand::N;  // integer ddambiguities
  using gtsam::symbol_shorthand::M;  // integer ambiguities for td cp without dd
  using gtsam::symbol_shorthand::A;
  using gtsam::symbol_shorthand::O;

  enum StateMeasSyncStatus {
    SYNCHRONIZED_I = 0,
    SYNCHRONIZED_J = 1,
    INTERPOLATED = 2,
    CACHED = 3,
    DROPPED = 4
  };

  struct StateMeasSyncResult {
    StateMeasSyncStatus status = StateMeasSyncStatus::DROPPED;
    bool foundI = false;
    size_t keyIndexI = 0;
    size_t keyIndexJ = 0;
    double timestampI = std::numeric_limits<double>::max();
    double timestampJ = std::numeric_limits<double>::max();
    double durationFromStateI = std::numeric_limits<double>::max();

    [[nodiscard]] bool stateJExist() const { return timestampJ < std::numeric_limits<double>::max(); }

    [[nodiscard]] bool stateIExist() const { return timestampI < std::numeric_limits<double>::max(); }

    [[nodiscard]] bool statesExist() const { return stateJExist() && stateIExist(); }
  };

  // Message type for sensor processing report
  struct SensorProcessingReport {
    double timestamp;
    std::string sensor_name;
    int measurements_processed;
    double processing_time_ms;
    double measurement_delay;
    bool observation_available;
  };

  // PPS message type
  struct PPS {
    double timestamp;
    int64_t count;
  };

  class IntegratorBase {
  protected:
    std::string integratorName_;
    std::string sensorName_;
    bool isPrimarySensor_ = false;

    fgo::core::ApplicationInterface* appPtr_{}; // Application interface
    std::shared_ptr<fgo::core::PublisherInterface<SensorProcessingReport>> pubSensorReport_;

    // Forward declared to avoid circular dependency
    fgo::graph::GraphBase *graphPtr_{};
    IntegratorBaseParamsPtr integratorBaseParamPtr_;
    uint64_t nState_{};
    double noOptimizationDuration_ = 0.;
    fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap lastKeyIndexTimestampMap_;
    std::vector<size_t> lastLagKeys_;

    /*
     *  Utilities
     */
    // std::unique_ptr<fgo::utils::MeasurementDelayCalculator> delayCalculator_;  // File doesn't exist
    std::shared_ptr<fgo::sensor::SensorCalibrationManager> sensorCalibManager_;

  public:

    /***
     * if a state out of the current lag size should be queried for e.g. delayed sensor observations, the solver
     * would crash, we filter these observations out by checking if the related states are alive
     * @param id state id as number (not gtsam identifier)
     * @return true if the state can be queried
     */
    bool checkStatePresentedInCurrentLag(size_t id) {
      if (lastLagKeys_.empty()) {
        appPtr_->getLogger().warn(integratorName_ + " checking state presence for key: " + std::to_string(id) +
                              " current key vector empty!");
        return true;
      }
      auto keyIter = std::find(lastLagKeys_.begin(), lastLagKeys_.end(), id);
      if (keyIter == lastLagKeys_.end()) {
        return false;
      } else
        return true;
    }

    /***
     * set attitude type from the parameter, different reference sensor may provide different types of attitude observations
     * @param typeStr must be one of rpy, yawroll, yawpitch, yaw, roll, pitch
     * @param type intern rotation type
     * @param name name for logging, usually the sensor name
     */
    static void setAttitudeType(const std::string &typeStr,
                                fgo::factor::AttitudeType &type,
                                fgo::core::LoggerInterface* logger = nullptr,
                                const std::string &name = "") {
      auto typeStrLower = boost::algorithm::to_lower_copy(typeStr);
      if (typeStrLower == "rpy")
        type = fgo::factor::AttitudeType::RPY;
      else if (typeStrLower == "yawroll")
        type = fgo::factor::AttitudeType::YAWROLL;
      else if (typeStrLower == "yawpitch")
        type = fgo::factor::AttitudeType::YAWPITCH;
      else if (typeStrLower == "yaw")
        type = fgo::factor::AttitudeType::YAW;
      else if (typeStrLower == "roll")
        type = fgo::factor::AttitudeType::ROLL;
      else if (typeStrLower == "pitch")
        type = fgo::factor::AttitudeType::PITCH;
      else if (logger)
        logger->warn(name + " improper attitude type: " + typeStrLower);
    }

    /***
     * get attitude noise vector according to the attitude type, if the attitude only contain one component,
     * the dimension of the error should be adapted
     * @param noise3D original full noise vector
     * @param type type of the attitude
     * @return noise vector
     */
    static gtsam::Vector getAttitudeNoiseVector(const gtsam::Vector3 &noise3D,
                                                const fgo::factor::AttitudeType &type) {
      switch (type) {
        case fgo::factor::AttitudeType::RPY: {
          return noise3D;
        }
        case fgo::factor::AttitudeType::YAWPITCH: {
          return (gtsam::Vector2() << noise3D.y(), noise3D.z()).finished();
        }
        case fgo::factor::AttitudeType::YAWROLL: {
          return (gtsam::Vector2() << noise3D.x(), noise3D.z()).finished();
        }
        case fgo::factor::AttitudeType::YAW: {
          return (gtsam::Vector1() << noise3D.z()).finished();
        }
        case fgo::factor::AttitudeType::ROLL: {
          return (gtsam::Vector1() << noise3D.x()).finished();
        }
        case fgo::factor::AttitudeType::PITCH: {
          return (gtsam::Vector1() << noise3D.y()).finished();
        }
      }
      return noise3D; // Default fallback
    }

    /***
     * set velocity type from the parameter, different reference sensor may provide different types of velocity observations
     * @param typeStr must be one of 3d, 2d, y, x, z
     * @param type intern velocity type
     * @param name name for logging, usually the sensor name
     */
    static void setVelocityType(const std::string &typeStr,
                                fgo::factor::VelocityType &type,
                                fgo::core::LoggerInterface* logger = nullptr,
                                const std::string &name = "") {
      auto typeStrLower = boost::algorithm::to_lower_copy(typeStr);
      if (typeStrLower == "3d")
        type = fgo::factor::VelocityType::VEL3D;
      else if (typeStrLower == "2d")
        type = fgo::factor::VelocityType::VEL2D;
      else if (typeStrLower == "y")
        type = fgo::factor::VelocityType::VELY;
      else if (typeStrLower == "x")
        type = fgo::factor::VelocityType::VELX;
      else if (typeStrLower == "z")
        type = fgo::factor::VelocityType::VELZ;
      else if (logger)
        logger->warn(name + " improper velocity type: " + typeStrLower);
    }

    /***
     * get velocity noise vector according to the velocity type, if the velocity only contain one component,
     * the dimension of the error should be adapted
     * @param noise3D original full noise vector
     * @param type type of the velocity
     * @return noise vector
     */
    static gtsam::Vector getVelocityNoiseVector(const gtsam::Vector3 &noise3D,
                                                const fgo::factor::VelocityType &type) {
      switch (type) {
        case fgo::factor::VelocityType::VEL3D: {
          return noise3D;
        }
        case fgo::factor::VelocityType::VEL2D: {
          return (gtsam::Vector2() << noise3D.x(), noise3D.y()).finished();
        }
        case fgo::factor::VelocityType::VELX: {
          return (gtsam::Vector1() << noise3D.x()).finished();
        }
        case fgo::factor::VelocityType::VELY: {
          return (gtsam::Vector1() << noise3D.y()).finished();
        }
        case fgo::factor::VelocityType::VELZ: {
          return (gtsam::Vector1() << noise3D.z()).finished();
        }
      }
      return noise3D; // Default fallback
    }

    /***
     * set sensor observation frame
     * @param frameStr must be one of body, ned, enu, ecef, default is ned
     * @param frame intern frame variable
     * @param type name for logging, usually the sensor name
     */
    static void setSensorFrameFromParam(const std::string &frameStr,
                                        fgo::factor::MeasurementFrame &frame,
                                        fgo::core::LoggerInterface* logger = nullptr,
                                        const std::string &type = "") {
      auto frameStrLower = boost::algorithm::to_lower_copy(frameStr);
      if (frameStrLower == "body")
        frame = fgo::factor::MeasurementFrame::BODY;
      else if (frameStrLower == "ned")
        frame = fgo::factor::MeasurementFrame::NED;
      else if (frameStrLower == "enu")
        frame = fgo::factor::MeasurementFrame::ENU;
      else if (frameStrLower == "ecef")
        frame = fgo::factor::MeasurementFrame::ECEF;
      else {
        frame = fgo::factor::MeasurementFrame::NED;
        if (logger)
          logger->warn(type + " improper frame: " + frameStrLower + " setting as NED");
      }
    }

    /***
     * set the noise model from parameter
     * @param modelStr name of the noise model
     * @param model intern model variable
     * @param type name for logging, usually the sensor name
     */
    static void setNoiseModelFromParam(const std::string &modelStr,
                                       fgo::data::NoiseModel &model,
                                       fgo::core::LoggerInterface* logger = nullptr,
                                       const std::string &type = "") {
      auto modelStrLower = boost::algorithm::to_lower_copy(modelStr);
      if (modelStrLower == "gaussian")
        model = data::NoiseModel::GAUSSIAN;
      else if (modelStrLower == "huber")
        model = data::NoiseModel::HUBER;
      else if (modelStrLower == "cauchy")
        model = data::NoiseModel::CAUCHY;
      else if (modelStrLower == "dcs")
        model = data::NoiseModel::DCS;
      else if (modelStrLower == "tukey")
        model = data::NoiseModel::Tukey;
      else if (modelStrLower == "gemanmcclure")
        model = data::NoiseModel::GemanMcClure;
      else if (modelStrLower == "welsch")
        model = data::NoiseModel::Welsch;
      else {
        model = data::NoiseModel::GAUSSIAN;
        if (logger)
          logger->warn(type + " improper noise model: " + modelStrLower + " setting as Gaussian");
      }
    }


  protected:
    void addNavAttitudeFactor(const gtsam::Key &poseKey,
                             const gtsam::Rot3 &attitudeMeasured, const gtsam::Vector3 &attitudeVar,
                             factor::AttitudeType type = factor::AttitudeType::RPY);

    void addGPInterpolatedNavAttitudeFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI,
                                           const gtsam::Key &omegaKeyI,
                                           const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ,
                                           const gtsam::Key &omegaKeyJ,
                                           const gtsam::Rot3 &attitudeMeasured, const gtsam::Vector3 &attitudeVar,
                                           const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                           factor::AttitudeType type = factor::AttitudeType::RPY);

    void addNavVelocityFactor(const gtsam::Key &poseKey, const gtsam::Key &velKey,
                             const gtsam::Vector3 &velMeasured, const gtsam::Vector3 &velMeasuredVar,
                             factor::VelocityType type = factor::VelocityType::VEL3D);

    void addGPInterpolatedNavVelocityFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI,
                                            const gtsam::Key &omegaKeyI,
                                            const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ,
                                            const gtsam::Key &omegaKeyJ,
                                            const gtsam::Vector3 &velMeasured, const gtsam::Vector3 &velMeasuredVar,
                                            const gtsam::Vector3 &lb,
                                            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                            factor::VelocityType type = factor::VelocityType::VEL3D);

    void addNavPoseFactor(const gtsam::Key &poseKey, const gtsam::Pose3 &poseMeasured, const gtsam::Vector6 &poseVar);

    void
    addGPinteporatedNavPoseFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI, const gtsam::Key &omegaKeyI,
                                  const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ, const gtsam::Key &omegaKeyJ,
                                  const gtsam::Pose3 &poseMeasured, const gtsam::Vector6 &poseVar,
                                  const std::shared_ptr<fgo::models::GPInterpolator> &interpolator);

  public:
    typedef std::shared_ptr<IntegratorBase> Ptr;
    typedef std::map<std::string, Ptr> IntegratorMap;

    explicit IntegratorBase() = default;

    /***
     * initialize the integrator instance, used in graphBase
     * @param app application interface
     * @param graphPtr graph pointer
     * @param integratorName name of the sensor
     * @param isPrimarySensor indicate if the sensor is used as primary sensor for sensor-centric integration
     */
    virtual void initialize(fgo::core::ApplicationInterface &app,
                            fgo::graph::GraphBase &graphRef,
                            const std::string &integratorName,
                            bool isPrimarySensor = false);

    fgo::integrator::param::IntegratorBaseParamsPtr getIntegratorBaseParamPtr() { return integratorBaseParamPtr_; };

    virtual ~IntegratorBase() = default;

    /***
     * convert and formulate the sensor observations and add them into the graph
     * @param timestampGyroMap gyro measurements could be used to correct the leverarm effect
     * @param stateIDAccMap acc measurement could be used for GP interpolation
     * @param currentKeyIndexTimestampMap current state keyindex and timestamp map, used to query states
     * @param timePredStates some sensor observations may need current system state to be pre-processed
     * @param values prior values
     * @param keyTimestampMap reverse of keyindexTimestampMap
     * @param relatedKeys a vector of related states for all observations, used in the solver
     * @return
     */
    virtual bool addFactors(
        const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
        const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> &stateIDAccMap,
        const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
        std::vector<std::pair<fgo::core::TimeStamp, fgo::data::State>> &timePredStates,
        gtsam::Values &values,
        fgo::solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
        gtsam::KeyVector &relatedKeys
    ) { return true; };

    virtual std::map<uint64_t, double> factorizeAsPrimarySensor() { return {}; };

    virtual void bufferIMUData(double imuTimestamp, const gtsam::Vector6 acc) {};

    virtual bool checkZeroVelocity() { return false; };

    virtual bool checkHasMeasurements() { return true; };

    virtual void cleanBuffers() {};

    virtual void notifyOptimization(double noOptimizationDuration) {
      noOptimizationDuration_ = 0.;
    };

    /***
     * some algorithms, such as odometries, need optimized system state to update keyframes
     * @param result gtsam result containing all optimized state variables
     * @param marginals gtsam marginal containing all optimized state variables
     * @param keyIndexTimestampMap current state keyindex and timestamp map, used to query states
     * @param optState current optimized state
     * @return
     */
    virtual bool fetchResult(
        const gtsam::Values &result,
        const gtsam::Marginals &marginals,
        const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
        fgo::data::State &optState
    ) { return true; };

    virtual void dropMeasurementBefore(double timestamp) {};

    std::string getName() { return integratorName_; }

    [[nodiscard]] bool isPrimarySensor() const { return isPrimarySensor_; }

    /***
     * find acc measurement according to the state id
     * @param idStateI state id
     * @param stateIDAccMap map
     * @return if a measurement could be found, associated acc measurement
     */
    std::tuple<bool, gtsam::Vector6, bool, gtsam::Vector6> findAccelerationToState(
        size_t idStateI,
        const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> &stateIDAccMap) {
      bool foundAccI = false, foundAccJ = false;
      gtsam::Vector6 accI = (stateIDAccMap.end() - 1)->second, accJ = stateIDAccMap.back().second;
      auto accMapIterator = stateIDAccMap.begin();
      while (accMapIterator != stateIDAccMap.end()) {
        if (accMapIterator->first == idStateI) {
          accI = accMapIterator->second;
          accMapIterator++;
          foundAccI = true;
          if (accMapIterator == stateIDAccMap.end())
            accJ = (accMapIterator - 1)->second;
          else {
            foundAccJ = true;
            accJ = accMapIterator->second;
          }
          break;
        }
        accMapIterator++;
      }
      if (!foundAccI || !foundAccJ)
        appPtr_->getLogger().warn(integratorName_ + " can find accI? " + std::to_string(foundAccI) + 
                              " or accJ? " + std::to_string(foundAccJ));
      return {foundAccI, accI, foundAccJ, accJ};
    }

    /***
     * find gyro measurement according to the timestamp
     * @param timestamp measurement timestamp
     * @param timestampGyroMap map
     * @return if a measurement could be found, associated gyro measurement
     */
    std::tuple<bool, gtsam::Vector3> findOmegaToMeasurement(
        double timestamp,
        const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap) {
      gtsam::Vector3 this_gyro = timestampGyroMap.back().second;
      bool find_gyro = false;
      auto timeGyroIter = timestampGyroMap.rbegin();
      while (timeGyroIter != timestampGyroMap.rend()) {
        const auto gyroTimediff = abs(timeGyroIter->first - timestamp);
        if (gyroTimediff < integratorBaseParamPtr_->IMUSensorSyncTimeThreshold) {
          find_gyro = true;
          this_gyro = timeGyroIter->second;
          break;
        }
        timeGyroIter++;
      }
      if (!find_gyro)
        appPtr_->getLogger().warn(integratorName_ + ": cannot find a gyro, using last one ...");
      return {find_gyro, this_gyro};
    }

    /***
     * synchronize sensor measurement with state, find the nearest state or interpolate
     * @param correctedTimestampMeas corrected measurement timestamp
     * @param varIDTimestampMap state id and timestamp map
     * @param paramPtr integrator parameters
     * @return sync result with status and state indices
     */
    static StateMeasSyncResult stateMeasSynchronize(
        double correctedTimestampMeas,
        const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &varIDTimestampMap,
        const IntegratorBaseParamsPtr &paramPtr) {
      StateMeasSyncResult result;

      if (varIDTimestampMap.empty())
        return result;

      auto varIDTimeIter = varIDTimestampMap.rbegin();
      while (varIDTimeIter != varIDTimestampMap.rend()) {
        const auto varGNSSdt = correctedTimestampMeas - varIDTimeIter->second;

        if (paramPtr->StateMeasSyncLowerBound <= varGNSSdt && varGNSSdt <= paramPtr->StateMeasSyncUpperBound) {
          result.foundI = true;
          if (varGNSSdt >= 0 && varIDTimeIter != varIDTimestampMap.rbegin()) { // Found sync I
            result.keyIndexI = varIDTimeIter->first;
            result.keyIndexJ = result.keyIndexI + 1;
            result.timestampI = varIDTimeIter->second;
            result.timestampJ = std::prev(varIDTimeIter)->second;
            result.durationFromStateI = abs(varGNSSdt);
            result.status = StateMeasSyncStatus::SYNCHRONIZED_I;
            return result;
          } else { // Found sync J
            result.keyIndexJ = varIDTimeIter->first;
            result.keyIndexI = result.keyIndexJ - 1;
            result.timestampJ = varIDTimeIter->second;
            result.timestampI = std::next(varIDTimeIter)->second;
            result.durationFromStateI = result.timestampJ - result.timestampI - abs(varGNSSdt);
            result.status = StateMeasSyncStatus::SYNCHRONIZED_J;
            return result;
          }
        }

        // If we go here, means, the meas is in between!
        if (varIDTimeIter->second < correctedTimestampMeas) {
          result.keyIndexI = varIDTimeIter->first;
          result.timestampI = varIDTimeIter->second;
          result.durationFromStateI = varGNSSdt;
          result.keyIndexJ = result.keyIndexI + 1;
          result.timestampJ = std::prev(varIDTimeIter)->second;
          result.foundI = true;
          result.status = StateMeasSyncStatus::INTERPOLATED;
          return result;
        }
        varIDTimeIter++;
      }
      return result;
    }
  };
}
#endif //ONLINE_FGO_CORE_INTEGRATIONBASE_H
