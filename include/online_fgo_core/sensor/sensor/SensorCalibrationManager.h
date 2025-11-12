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
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include "utils/ROSParameter.h"
#include "data/DataTypesFGO.h"

namespace fgo::sensor {

  struct SensorParameter {
    std::string sensorName;
    std::string baseFrame;
    gtsam::Rot3 preRotation;
    gtsam::Pose3 fromBase;  // transformation w.r.t. of base frame i.a. T^sensor_base
    std::map<std::string, gtsam::Pose3> toOthersMap;  // transformation from this sensor's frame to other's i.a. T^other_sensor
    std::map<std::string, std::vector<double>> intrinsicsRawMap; // todo
  };

  /**
   * This class is used to manage all sensor calibration parameters including intrinsic and extrinsic
   * ToDo: get tf2 involved:
   * ToDO: - publishing static tf links if the a link does not exist
   * ToDo: - reading from tf trees if links are given
   */
  class SensorCalibrationManager {
  public:
    using Ptr = std::shared_ptr<SensorCalibrationManager>;

    explicit SensorCalibrationManager(rclcpp::Node &node, const std::string &param_prefix = "") : rosPtr_(&node) {
      RCLCPP_INFO(rosPtr_->get_logger(),
                  "SensorCalibrationManager: initializing sensor calibration parameter manager from configs...");

      this->initialize(param_prefix);

      RCLCPP_INFO_STREAM(rosPtr_->get_logger(),
                         "SensorCalibrationManager: initializing sensor calibration parameter manager with "
                           << sensorMap_.size() << " sensors is done ...");
    };

    ~SensorCalibrationManager() = default;

    gtsam::Rot3 getPreRotation(const std::string &sensor) const {
      std::shared_lock lock(mutex_);
      const auto iter = sensorMap_.find(sensor);
      if (iter != sensorMap_.end()) {
        return sensorMap_.at(sensor).preRotation;
      } else
        RCLCPP_WARN_STREAM(rosPtr_->get_logger(),
                           "SensorCalibrationManager in getPreRotation: unknown sensor " << sensor);
      return {};
    }


    gtsam::Pose3 getTransformationFromBase(const std::string &sensor) const {
      std::shared_lock lock(mutex_);
      const auto iter = sensorMap_.find(sensor);
      if (iter != sensorMap_.end()) {
        return sensorMap_.at(sensor).fromBase;
      } else
        RCLCPP_WARN_STREAM(rosPtr_->get_logger(),
                           "SensorCalibrationManager in getTransformationFromBase: unknown sensor " << sensor);
      return {};
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
          RCLCPP_WARN_STREAM(rosPtr_->get_logger(),
                             "SensorCalibrationManager in getTransformationFromBaseToTarget: unknown target sensor "
                               << target);
        return trans;
      } else
        RCLCPP_WARN_STREAM(rosPtr_->get_logger(),
                           "SensorCalibrationManager in getTransformationFromBaseToTarget: unknown base sensor "
                             << target);
      return trans;
    }

    void initialize(const std::string &param_prefix) {
      std::unique_lock<std::shared_mutex> ul(mutex_);
      const auto param_ns = param_prefix + ".VehicleParameters";

      ::utils::RosParameter<std::vector<std::string>> sensors(param_ns + ".sensors", *rosPtr_);
      ::utils::RosParameter<std::string> base_frame(param_ns + ".baseFrame", *rosPtr_);
      baseFrame_ = base_frame.value();
      RCLCPP_INFO_STREAM(rosPtr_->get_logger(),
                         "SensorCalibrationManager: sensors used of " << param_prefix << " with base frame "
                                                                      << baseFrame_ << " are: ");
      for (const auto &sensor: sensors.value()) {
        const auto param_ns_sensor = param_ns + "." + sensor;
        RCLCPP_INFO_STREAM(rosPtr_->get_logger(), "* sensor: " << sensor);
        SensorParameter sensor_param;
        sensor_param.sensorName = sensor;
        sensor_param.baseFrame = base_frame.value();
        std::vector<double> trans_from_base = {0., 0., 0.};
        std::vector<double> rot_from_base = {0., 0., 0.};
        std::vector<double> rot_pre = {0., 0., 0.};
        ::utils::RosParameter<std::vector<double>> trans_base(param_ns_sensor + ".transFromBase", trans_from_base,
                                                              *rosPtr_);
        ::utils::RosParameter<std::vector<double>> rot_base(param_ns_sensor + ".rotFromBase", rot_from_base, *rosPtr_);
        ::utils::RosParameter<std::vector<double>> rot_self(param_ns_sensor + ".preRotate", rot_pre, *rosPtr_);

        trans_from_base = trans_base.value();
        rot_from_base = rot_base.value();
        rot_pre = rot_self.value();

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
          RCLCPP_WARN_STREAM(rosPtr_->get_logger(),
                             "* sensor" << sensor << " was given invalid rotation parameters in size "
                                        << rot_from_base.size());
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
          RCLCPP_WARN_STREAM(rosPtr_->get_logger(),
                             "* sensor" << sensor << " was given invalid pre rotation parameters in size "
                                        << rot_pre.size());
        }

        sensor_param.fromBase = gtsam::Pose3(rot, trans);
        sensorMap_.insert(std::make_pair(sensor, sensor_param));
        RCLCPP_INFO_STREAM(rosPtr_->get_logger(),
                           "** param: " << std::fixed
                                        << "\n***  trans:" << sensor_param.fromBase.translation()
                                        << "\n*** rpy: "
                                        << sensor_param.fromBase.rotation().rpy() * constants::rad2deg);
        RCLCPP_INFO(rosPtr_->get_logger(), "******");
      }

      for (auto &[sen1_name, sen1_param]: sensorMap_) {
        for (const auto &[sen2_name, sen2_param]: sensorMap_) {
          if (sen1_name == sen2_name)
            continue;

          // T^sen2_sen1 = T^sen2_base * inverse(T_sen1_base);
          const auto trans_between = sen2_param.fromBase.transformPoseFrom(sen1_param.fromBase.inverse());
          sen1_param.toOthersMap.insert(std::make_pair(sen2_name, trans_between));
          //RCLCPP_INFO_STREAM(rosPtr_->get_logger(),
          //                   "** param from " << sen1_name << " to: " << sen2_name << std::fixed
          //                                   << "\n***  trans:" << trans_between.translation()
          //                                   << "\n*** rpy: " << trans_between.rotation().rpy() * constants::rad2deg);
          //RCLCPP_INFO(rosPtr_->get_logger(), "******");
        }
      }

      ::utils::RosParameter<std::string> intrinsic_json(param_ns + ".intrinsic_json_path", "", *rosPtr_);

      if (!intrinsic_json.value().empty())
        this->readIntrinsicsFromJSON(intrinsic_json.value());
    }

    void readIntrinsicsFromJSON(const std::string &json_file) {
      // ToDo
    }

    SensorParameter getSensorParameter(const std::string &sensor) const {
      const auto iter = sensorMap_.find(sensor);
      if (iter == sensorMap_.end()) {
        RCLCPP_WARN_STREAM(rosPtr_->get_logger(),
                           "SensorCalibrationManager: getSensorParameter unknown sensor " << sensor);
        return {};
      }
      return sensorMap_.at(sensor);
    }

    void printSensorCalibParam(const std::string &sensor) {
      const auto iter = sensorMap_.find(sensor);
      if (iter == sensorMap_.end()) {
        RCLCPP_WARN_STREAM(rosPtr_->get_logger(),
                           "SensorCalibrationManager: printSensorCalibParam unknown sensor " << sensor);
        return;
      }

      const auto &sensorParam = sensorMap_.at(sensor);
      std::cout << "********************** PRINTING SENSOR CALIBRATION PARAMETER of " << sensor
                << " **********************" << std::endl;
      std::cout << "********************** Extrinsic **********************" << std::endl;
      std::cout << "* base frame " << baseFrame_ << std::endl;
      std::cout << "* fromBase: " << std::endl;
      std::cout << "** trans: \n" << sensorParam.fromBase.translation() << std::endl;
      std::cout << "** rot rpy: \n " << sensorParam.fromBase.rotation().rpy() * constants::rad2deg << std::endl;
      std::cout << "* preRotate: " << std::endl;
      std::cout << "** rot rpy: \n " << sensorParam.preRotation.rpy() * constants::rad2deg << std::endl;

      for (const auto &[sen_name, sen_trans]: sensorParam.toOthersMap) {
        std::cout << "* to sensor " << sen_name << std::endl;
        std::cout << "** trans: \n" << sen_trans.translation() << std::endl;
        std::cout << "** rot rpy: \n " << sen_trans.rotation().rpy() * constants::rad2deg << std::endl;
      }

      if (!sensorParam.intrinsicsRawMap.empty()) {
        std::cout << "********************** Extrinsic **********************" << std::endl;
        for (const auto &[int_name, int_param]: sensorParam.intrinsicsRawMap) {
          std::cout << "* " << int_name << ": ";
          for (const auto &value: int_param)
            std::cout << std::fixed << value << ", ";
          std::cout << std::endl;
        }
      }
    }


  private:
    rclcpp::Node *rosPtr_;
    mutable std::shared_mutex mutex_;
    std::string baseFrame_;
    std::map<std::string, SensorParameter> sensorMap_;
  };
}


#endif //ONLINE_FGO_SENSORCALIBRATIONMANAGER_H
