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
//

//
// Created by haoming on 11.07.24.
//

#ifndef ONLINE_FGO_SENSORCALIBRATIONMANAGER_H
#define ONLINE_FGO_SENSORCALIBRATIONMANAGER_H
#pragma once

#include <iostream>
#include <string>
#include <map>
#include <memory>
#include <shared_mutex>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include "online_fgo_core/interface/LoggerInterface.h"
#include "online_fgo_core/interface/ParameterInterface.h"
#include "online_fgo_core/data/DataTypesFGO.h"

namespace fgo::sensor {

  struct SensorParameter {
    std::string sensorName;
    std::string baseFrame;
    gtsam::Rot3 preRotation;
    gtsam::Pose3 fromBase;  // transformation w.r.t. of base frame i.a. T^sensor_base
    std::map<std::string, gtsam::Pose3> toOthersMap;  // transformation from this sensor's frame to other's i.a. T^other_sensor
    std::map<std::string, std::vector<double>> intrinsicsRawMap; // todo
    
    SensorParameter() : preRotation(gtsam::Rot3::identity()), 
                        fromBase(gtsam::Pose3::identity()) {}
  };

  /**
   * This class is used to manage all sensor calibration parameters including intrinsic and extrinsic
   * ROS-agnostic version using ApplicationInterface
   */
  class SensorCalibrationManager {
  public:
    using Ptr = std::shared_ptr<SensorCalibrationManager>;

    explicit SensorCalibrationManager(fgo::core::LoggerInterface &logger,
                                     fgo::core::ParameterInterface &params,
                                     const std::string &param_prefix = "") 
        : logger_(&logger), params_(&params) {
      logger_->info("SensorCalibrationManager: initializing sensor calibration parameter manager from configs...");

      this->initialize(param_prefix);

      logger_->info("SensorCalibrationManager: initializing sensor calibration parameter manager with " +
                   std::to_string(sensorMap_.size()) + " sensors is done ...");
    }

    ~SensorCalibrationManager() = default;

    gtsam::Rot3 getPreRotation(const std::string &sensor) const {
      std::shared_lock lock(mutex_);
      const auto iter = sensorMap_.find(sensor);
      if (iter != sensorMap_.end()) {
        return sensorMap_.at(sensor).preRotation;
      } else {
        logger_->warn("SensorCalibrationManager in getPreRotation: unknown sensor " + sensor);
      }
      return gtsam::Rot3::identity();
    }


    gtsam::Pose3 getTransformationFromBase(const std::string &sensor) const {
      std::shared_lock lock(mutex_);
      const auto iter = sensorMap_.find(sensor);
      if (iter != sensorMap_.end()) {
        return sensorMap_.at(sensor).fromBase;
      } else {
        logger_->warn("SensorCalibrationManager in getTransformationFromBase: unknown sensor " + sensor);
      }
      return gtsam::Pose3::identity();
    }

    gtsam::Pose3 getTransformationFromBaseToTarget(const std::string &base,
                                                   const std::string &target) const {
      gtsam::Pose3 trans;
      std::shared_lock lock(mutex_);
      const auto iter = sensorMap_.find(base);
      if (iter != sensorMap_.end()) {
        const auto &baseSensor = sensorMap_.at(base);
        const auto iter_sub = baseSensor.toOthersMap.find(target);
        if (iter_sub != baseSensor.toOthersMap.end())
          trans = baseSensor.toOthersMap.at(target);
        else
          logger_->warn("SensorCalibrationManager in getTransformationFromBaseToTarget: unknown target sensor " + target);
        return trans;
      } else {
        logger_->warn("SensorCalibrationManager in getTransformationFromBaseToTarget: unknown base sensor " + base);
      }
      return trans;
    }

    void initialize(const std::string &param_prefix) {
      std::unique_lock<std::shared_mutex> ul(mutex_);
      const auto param_ns = param_prefix + ".VehicleParameters";

      auto sensors = params_->getStringArray(param_ns + ".sensors", {});
      baseFrame_ = params_->getString(param_ns + ".baseFrame", "base_link");
      
      logger_->info("SensorCalibrationManager: sensors used of " + param_prefix + 
                   " with base frame " + baseFrame_ + " are: ");
      
      for (const auto &sensor: sensors) {
        const auto param_ns_sensor = param_ns + "." + sensor;
        logger_->info("* sensor: " + sensor);
        SensorParameter sensor_param;
        sensor_param.sensorName = sensor;
        sensor_param.baseFrame = baseFrame_;
        
        std::vector<double> trans_from_base = params_->getDoubleArray(
            param_ns_sensor + ".transFromBase", {0., 0., 0.});
        std::vector<double> rot_from_base = params_->getDoubleArray(
            param_ns_sensor + ".rotFromBase", {0., 0., 0.});
        std::vector<double> rot_pre = params_->getDoubleArray(
            param_ns_sensor + ".preRotate", {0., 0., 0.});

        const auto trans = gtsam::Point3(trans_from_base.data());
        auto rot = gtsam::Rot3::identity();

        if (rot_from_base.size() == 3) {
          // rot parameter is given as roll pitch yaw in RAD!
          rot = gtsam::Rot3::RzRyRx(rot_from_base[0], rot_from_base[1], rot_from_base[2]);
        } else if (rot_from_base.size() == 4) {
          // rot parameter is given as quaternion as w x y z
          rot = gtsam::Rot3::Quaternion(rot_from_base[0], rot_from_base[1], rot_from_base[2], rot_from_base[3]);
        } else if (rot_from_base.size() == 9) {
          // rot parameter is given as rotation matrix
          rot = gtsam::Rot3(gtsam::Matrix33(rot_from_base.data()));
        } else {
          logger_->warn("* sensor " + sensor + " was given invalid rotation parameters in size " +
                       std::to_string(rot_from_base.size()));
        }

        if (rot_pre.size() == 3) {
          // rot parameter is given as roll pitch yaw in RAD!
          sensor_param.preRotation = gtsam::Rot3::RzRyRx(rot_pre[0], rot_pre[1], rot_pre[2]);
        } else if (rot_pre.size() == 4) {
          // rot parameter is given as quaternion as w x y z
          sensor_param.preRotation = gtsam::Rot3::Quaternion(rot_pre[0], rot_pre[1], rot_pre[2], rot_pre[3]);
        } else if (rot_pre.size() == 9) {
          // rot parameter is given as rotation matrix
          sensor_param.preRotation = gtsam::Rot3(gtsam::Matrix33(rot_pre.data()));
        } else {
          logger_->warn("* sensor " + sensor + " was given invalid pre rotation parameters in size " +
                       std::to_string(rot_pre.size()));
        }

        sensor_param.fromBase = gtsam::Pose3(rot, trans);
        sensorMap_.insert(std::make_pair(sensor, sensor_param));
        
        auto rpy_deg = sensor_param.fromBase.rotation().rpy() * constants::rad2deg;
        logger_->info("** param: trans:[" + 
                     std::to_string(sensor_param.fromBase.translation().x()) + "," +
                     std::to_string(sensor_param.fromBase.translation().y()) + "," +
                     std::to_string(sensor_param.fromBase.translation().z()) + "] rpy:[" +
                     std::to_string(rpy_deg.x()) + "," +
                     std::to_string(rpy_deg.y()) + "," +
                     std::to_string(rpy_deg.z()) + "]");
        logger_->info("******");
      }

      for (auto &[sen1_name, sen1_param]: sensorMap_) {
        for (const auto &[sen2_name, sen2_param]: sensorMap_) {
          if (sen1_name == sen2_name)
            continue;

          // T^sen2_sen1 = T^sen2_base * inverse(T_sen1_base);
          const auto trans_between = sen2_param.fromBase.transformPoseFrom(sen1_param.fromBase.inverse());
          sen1_param.toOthersMap.insert(std::make_pair(sen2_name, trans_between));
        }
      }

      auto intrinsic_json = params_->getString(param_ns + ".intrinsic_json_path", "");
      if (!intrinsic_json.empty())
        this->readIntrinsicsFromJSON(intrinsic_json);
    }

    void readIntrinsicsFromJSON(const std::string &json_file) {
      // ToDo
    }

    SensorParameter getSensorParameter(const std::string &sensor) const {
      const auto iter = sensorMap_.find(sensor);
      if (iter == sensorMap_.end()) {
        logger_->warn("SensorCalibrationManager: getSensorParameter unknown sensor " + sensor);
        return {};
      }
      return sensorMap_.at(sensor);
    }

    void printSensorCalibParam(const std::string &sensor) {
      const auto iter = sensorMap_.find(sensor);
      if (iter == sensorMap_.end()) {
        logger_->warn("SensorCalibrationManager: printSensorCalibParam unknown sensor " + sensor);
        return;
      }

      const auto &sensorParam = sensorMap_.at(sensor);
      logger_->info("********************** PRINTING SENSOR CALIBRATION PARAMETER of " + sensor +
                   " **********************");
      logger_->info("********************** Extrinsic **********************");
      logger_->info("* base frame " + baseFrame_);
      logger_->info("* fromBase: ");
      logger_->info("** trans: [" + 
                   std::to_string(sensorParam.fromBase.translation().x()) + "," +
                   std::to_string(sensorParam.fromBase.translation().y()) + "," +
                   std::to_string(sensorParam.fromBase.translation().z()) + "]");
      auto rpy_deg = sensorParam.fromBase.rotation().rpy() * constants::rad2deg;
      logger_->info("** rot rpy: [" + 
                   std::to_string(rpy_deg.x()) + "," +
                   std::to_string(rpy_deg.y()) + "," +
                   std::to_string(rpy_deg.z()) + "]");
      logger_->info("* preRotate: ");
      auto pre_rpy_deg = sensorParam.preRotation.rpy() * constants::rad2deg;
      logger_->info("** rot rpy: [" + 
                   std::to_string(pre_rpy_deg.x()) + "," +
                   std::to_string(pre_rpy_deg.y()) + "," +
                   std::to_string(pre_rpy_deg.z()) + "]");

      for (const auto &[sen_name, sen_trans]: sensorParam.toOthersMap) {
        logger_->info("* to sensor " + sen_name);
        logger_->info("** trans: [" + 
                     std::to_string(sen_trans.translation().x()) + "," +
                     std::to_string(sen_trans.translation().y()) + "," +
                     std::to_string(sen_trans.translation().z()) + "]");
        auto to_rpy_deg = sen_trans.rotation().rpy() * constants::rad2deg;
        logger_->info("** rot rpy: [" + 
                     std::to_string(to_rpy_deg.x()) + "," +
                     std::to_string(to_rpy_deg.y()) + "," +
                     std::to_string(to_rpy_deg.z()) + "]");
      }

      if (!sensorParam.intrinsicsRawMap.empty()) {
        logger_->info("********************** Intrinsic **********************");
        for (const auto &[int_name, int_param]: sensorParam.intrinsicsRawMap) {
          std::string param_str = "* " + int_name + ": ";
          for (const auto &value: int_param)
            param_str += std::to_string(value) + ", ";
          logger_->info(param_str);
        }
      }
    }


  private:
    fgo::core::LoggerInterface *logger_;
    fgo::core::ParameterInterface *params_;
    mutable std::shared_mutex mutex_;
    std::string baseFrame_;
    std::map<std::string, SensorParameter> sensorMap_;
  };
}


#endif //ONLINE_FGO_SENSORCALIBRATIONMANAGER_H
