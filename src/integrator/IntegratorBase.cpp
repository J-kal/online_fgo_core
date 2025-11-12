#include "online_fgo_core/integrator/IntegratorBase.h"
#include "online_fgo_core/graph/GraphBase.h"

namespace fgo::integrator {

  void IntegratorBase::initialize(fgo::core::ApplicationInterface &app,
                                  fgo::graph::GraphBase &graphRef,
                                  const std::string &integratorName,
                                  bool isPrimarySensor) {
    appPtr_ = &app;
    graphPtr_ = &graphRef;
    integratorName_ = integratorName;
    integratorBaseParamPtr_ = std::make_shared<fgo::integrator::param::IntegratorBaseParams>(
        graphPtr_->getParamPtr());
    isPrimarySensor_ = isPrimarySensor;

    sensorName_ = appPtr_->getParameters().getString("GNSSFGO." + integratorName + ".sensorName", "");

    appPtr_->getLogger().info("--------------------- " + integratorName + " with sensor name " + sensorName_ +
                          ": start base initialization... ---------------------");
    appPtr_->getLogger().info("------- IntegratorBase Parameters: -------");

    integratorBaseParamPtr_->IMUSensorSyncTimeThreshold =
        appPtr_->getParameters().getDouble("GNSSFGO.IntegratorBase.IMUSensorSyncTimeThreshold", 0.01);
    appPtr_->getLogger().info("IMUSensorSyncTimeThreshold: " +
                          std::to_string(integratorBaseParamPtr_->IMUSensorSyncTimeThreshold));

    integratorBaseParamPtr_->StateSensorSyncTimeThreshold =
        appPtr_->getParameters().getDouble("GNSSFGO.IntegratorBase.StateSensorSyncTimeThreshold", 0.01);
    appPtr_->getLogger().info("StateSensorSyncTimeThreshold: " +
                          std::to_string(integratorBaseParamPtr_->StateSensorSyncTimeThreshold));

    sensorCalibManager_ = graphRef.getSensorCalibManagerPtr();
    pubSensorReport_ = appPtr_->createPublisher<SensorProcessingReport>(
        "sensor_processing_report/" + integratorName_);

    sensorCalibManager_->printSensorCalibParam(sensorName_);
    appPtr_->getLogger().info("--------------------- " + integratorName +
                          ": base initialized... ---------------------");
  }

  void IntegratorBase::addNavAttitudeFactor(const gtsam::Key &poseKey,
                                           const gtsam::Rot3 &attitudeMeasured, const gtsam::Vector3 &attitudeVar,
                                           factor::AttitudeType type) {
    const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelAttitude,
                                                    getAttitudeNoiseVector(attitudeVar, type),
                                                    integratorBaseParamPtr_->robustParamAttitude,
                                                    "NavAttitude");
    graphPtr_->emplace_shared<fgo::factor::NavAttitudeFactor>(poseKey, attitudeMeasured, 
                                                              factor::MeasurementFrame::NED,
                                                              type, noiseModel);
  }

  void IntegratorBase::addGPInterpolatedNavAttitudeFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI,
                                                         const gtsam::Key &omegaKeyI,
                                                         const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ,
                                                         const gtsam::Key &omegaKeyJ,
                                                         const gtsam::Rot3 &attitudeMeasured, const gtsam::Vector3 &attitudeVar,
                                                         const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                                         factor::AttitudeType type) {
    const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelAttitude,
                                                    getAttitudeNoiseVector(attitudeVar, type),
                                                    integratorBaseParamPtr_->robustParamAttitude,
                                                    "GPInterpolatedNavAttitude");
    graphPtr_->emplace_shared<fgo::factor::GPInterpolatedNavAttitudeFactor>(poseKeyI, velKeyI, omegaKeyI,
                                                                            poseKeyJ, velKeyJ, omegaKeyJ,
                                                                            attitudeMeasured, 
                                                                            factor::MeasurementFrame::NED,
                                                                            type,
                                                                            noiseModel, interpolator,
                                                                            integratorBaseParamPtr_->AutoDiffGPInterpolatedFactor);
  }

  void IntegratorBase::addNavVelocityFactor(const gtsam::Key &poseKey, const gtsam::Key &velKey,
                                           const gtsam::Vector3 &velMeasured, const gtsam::Vector3 &velMeasuredVar,
                                           factor::VelocityType type) {
    const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelVelocity,
                                                    getVelocityNoiseVector(velMeasuredVar, type),
                                                    integratorBaseParamPtr_->robustParamVelocity,
                                                    "NavVelocity");
    graphPtr_->emplace_shared<fgo::factor::NavVelocityFactor>(poseKey, velKey, velMeasured,
                                                              gtsam::Vector3::Zero(), // angularVelocity
                                                              gtsam::Vector3::Zero(), // lever arm
                                                              integratorBaseParamPtr_->velocityFrame, 
                                                              type,
                                                              noiseModel);
  }

  void IntegratorBase::addGPInterpolatedNavVelocityFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI,
                                                          const gtsam::Key &omegaKeyI,
                                                          const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ,
                                                          const gtsam::Key &omegaKeyJ,
                                                          const gtsam::Vector3 &velMeasured, const gtsam::Vector3 &velMeasuredVar,
                                                          const gtsam::Vector3 &lb,
                                                          const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                                          factor::VelocityType type) {
    const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelVelocity,
                                                    getVelocityNoiseVector(velMeasuredVar, type),
                                                    integratorBaseParamPtr_->robustParamVelocity,
                                                    "GPInterpolatedNavVelocity");
    graphPtr_->emplace_shared<fgo::factor::GPInterpolatedNavVelocityFactor>(poseKeyI, velKeyI, omegaKeyI, poseKeyJ,
                                                                            velKeyJ, omegaKeyJ,
                                                                            velMeasured, lb,
                                                                            integratorBaseParamPtr_->velocityFrame,
                                                                            type,
                                                                            noiseModel, interpolator, true);
  }

  void IntegratorBase::addNavPoseFactor(const gtsam::Key &poseKey, const gtsam::Pose3 &poseMeasured, const gtsam::Vector6 &poseVar) {
    const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelOdomPose,
                                                    poseVar,
                                                    integratorBaseParamPtr_->robustParamOdomPose,
                                                    "addNavPoseFactor");
    graphPtr_->emplace_shared<fgo::factor::NavPoseFactor>(poseKey, poseMeasured, noiseModel);
  }

  void IntegratorBase::addGPinteporatedNavPoseFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI, const gtsam::Key &omegaKeyI,
                                                    const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ, const gtsam::Key &omegaKeyJ,
                                                    const gtsam::Pose3 &poseMeasured, const gtsam::Vector6 &poseVar,
                                                    const std::shared_ptr<fgo::models::GPInterpolator> &interpolator) {
    const auto noiseModel = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelOdomPose,
                                                    poseVar,
                                                    integratorBaseParamPtr_->robustParamOdomPose,
                                                    "addNavPoseFactor");
    graphPtr_->emplace_shared<fgo::factor::GPInterpolatedNavPoseFactor>(poseKeyI, velKeyI, omegaKeyI, poseKeyJ,
                                                                        velKeyJ, omegaKeyJ,
                                                                        poseMeasured,
                                                                        interpolator, noiseModel,
                                                                        integratorBaseParamPtr_->AutoDiffGPInterpolatedFactor);
  }

}