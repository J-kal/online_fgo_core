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
//  Author: Haoming Zhang (haoming.zhang@rwth-aachen.de)
//
//

//
// Created by haoming on 09.06.24.
//

#ifndef ONLINE_FGO_ROBOGNSSPARSER_H
#define ONLINE_FGO_ROBOGNSSPARSER_H
#pragma once

#include <robognss_msgs/msg/pva.hpp>
#include <robognss_msgs/msg/pvt.hpp>
#include <robognss_msgs/msg/obs_extended.hpp>
#include <robognss_msgs/msg/sat_header.hpp>
#include <robognss_msgs/msg/gnss_header.hpp>
#include <robognss_msgs/msg/gnss_raw_header.hpp>
#include <robognss_msgs/msg/meas_epoch_pre_processed.hpp>
#include <robognss_msgs/msg/meas_epoch_pre_processed_multi_ant.hpp>

#include "GNSSDataParser.h"

namespace fgo::sensor::gnss {
  using namespace fgo::data;
  namespace robognss = robognss_msgs::msg;

  inline std::tuple<fgo::data::PVASolution, fgo::data::State>
  parseRoboGNSSPVAMsg(const robognss::PVA &pvaMsg,
                      const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    const auto thisPVATime = rclcpp::Time(pvaMsg.header.stamp.sec, pvaMsg.header.stamp.nanosec, RCL_ROS_TIME);
    fgo::data::PVASolution pva;
    pva.wnc = pvaMsg.gnss_header.wnc;
    pva.tow = pvaMsg.gnss_header.tow;
    pva.sol_age = pvaMsg.gnss_header.sol_age;
    pva.diff_age = pvaMsg.gnss_header.diff_age;
    pva.timestamp = thisPVATime;
    pva.has_heading = true;
    pva.has_velocity = true;
    pva.has_roll_pitch = true;
    pva.error = pvaMsg.gnss_header.sol_status;
    pva.llh = (gtsam::Vector3() << pvaMsg.lat_deg * fgo::constants::deg2rad,
                                   pvaMsg.lon_deg * fgo::constants::deg2rad,
                                   pvaMsg.height + pva.undulation).finished();
    pva.xyz_ecef = fgo::utils::llh2xyz(pva.llh);
    const auto eRn = gtsam::Rot3(fgo::utils::nedRe_Matrix(pva.xyz_ecef)).inverse();
    pva.xyz_var = (gtsam::Vector3() << std::pow(pvaMsg.pos_std.x, 2),
                                       std::pow(pvaMsg.pos_std.y, 2),
                                       std::pow(pvaMsg.pos_std.z, 2)).finished();

    pva.nRe = gtsam::Rot3(utils::nedRe_Matrix_asLLH(pva.llh));

    switch (pvaMsg.att.attitude_frame) {
      case robognss::Attitude::ECEF: {
        pva.rot_ecef = gtsam::Rot3::RzRyRx(pvaMsg.att.roll, pvaMsg.att.pitch, pvaMsg.att.yaw);
        pva.rot_var = (gtsam::Vector3() << std::pow(pvaMsg.att.roll_std, 2),
                                           std::pow(pvaMsg.att.pitch_std, 2),
                                           std::pow(pvaMsg.att.yaw_std, 2)).finished();
        pva.rot_n = pva.nRe * pva.rot_ecef;

        //Todo: do we need those?
        pva.heading = pva.rot_n.yaw();
        pva.heading_var = pva.rot_var.z();

        break;
      }
      case robognss::Attitude::NED: {
        pva.rot_n = gtsam::Rot3::RzRyRx(pvaMsg.att.roll, pvaMsg.att.pitch, pvaMsg.att.yaw);
        pva.rot_var = (gtsam::Vector3() << std::pow(pvaMsg.att.roll_std, 2),
          std::pow(pvaMsg.att.pitch_std, 2),
          std::pow(pvaMsg.att.yaw_std, 2)).finished();
        pva.rot_ecef = pva.nRe.inverse() * pva.rot_n;
        pva.heading = pva.rot_n.yaw();
        pva.heading_var = pva.rot_var.z();
        break;
      }
    }
    pva.has_velocity = true;
    pva.has_velocity_3D = true;

    switch (pvaMsg.vel.vel_frame) {
      case robognss::Velocity::ECEF: {
        pva.vel_ecef = (gtsam::Vector3() << pvaMsg.vel.vx, pvaMsg.vel.vy, pvaMsg.vel.vz).finished();
        pva.vel_n = pva.nRe.rotate(pva.vel_ecef);
        pva.vel_var = (gtsam::Vector3() << std::pow(pvaMsg.vel.vx_std, 2),
                                           std::pow(pvaMsg.vel.vy_std, 2),
                                           std::pow(pvaMsg.vel.vz_std, 2)).finished();
        break;
      }
      case robognss::Velocity::NED: {
        pva.vel_n = (gtsam::Vector3() << pvaMsg.vel.vx, pvaMsg.vel.vy, pvaMsg.vel.vz).finished();
        pva.vel_ecef = pva.nRe.inverse().rotate(pva.vel_n);
        pva.vel_var = pva.nRe.inverse().rotate((gtsam::Vector3() << std::pow(pvaMsg.vel.vx_std, 2),
                                                                    std::pow(pvaMsg.vel.vy_std, 2),
                                                                    std::pow(pvaMsg.vel.vz_std, 2)).finished());
        break;
      }
      case robognss::Velocity::ENU: {
        pva.vel_n = (gtsam::Vector3() << pvaMsg.vel.vy, pvaMsg.vel.vx, -pvaMsg.vel.vz).finished();
        const auto enuRe = gtsam::Rot3(utils::enuRe_Matrix_asLLH(pva.llh));
        pva.vel_ecef = enuRe.inverse().rotate(pva.vel_n);
        pva.vel_var = enuRe.inverse().rotate((gtsam::Vector3() << std::pow(pvaMsg.vel.vy_std, 2),
                                                                  std::pow(pvaMsg.vel.vx_std, 2),
                                                                  std::pow(pvaMsg.vel.vz_std, 2)).finished());
        break;
      }
      case robognss::Velocity::BODY: {
        pva.vel_ecef = pva.rot_ecef.rotate((gtsam::Vector3() << pvaMsg.vel.vx, pvaMsg.vel.vy, pvaMsg.vel.vz).finished());
        pva.vel_n = pva.nRe.rotate(pva.vel_ecef);
        pva.vel_var = (gtsam::Vector3() << std::pow(pvaMsg.vel.vx_std, 2),
          std::pow(pvaMsg.vel.vy_std, 2),
          std::pow(pvaMsg.vel.vz_std, 2)).finished();
        pva.vel_var = pva.rot_ecef.rotate(pva.vel_var);
        break;
      }
    }

    pva.clock_bias = pvaMsg.rx_clk_bias;
    pva.clock_drift = pvaMsg.rx_clk_drift;
    pva.clock_bias_var = std::pow(pvaMsg.rx_clk_bias_std, 2);
    pva.clock_drift_var = std::pow(pvaMsg.rx_clk_drift_std, 2);

    pva.num_sat = pvaMsg.gnss_header.num_sat_all;
    pva.num_sat_used = pvaMsg.gnss_header.num_sat_used;
    pva.num_sat_used_l1 = pvaMsg.gnss_header.num_sat_l1e1;
    pva.num_sat_used_multi = pvaMsg.gnss_header.num_sat_multi;

    pva.type = fgo::utils::GNSS::getSBFSolutionType(pvaMsg.gnss_header.sol_status, pvaMsg.gnss_header.mode);
    return {pva, PVASolutionToState(pva, leverArm)};
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State>
  parseRoboGNSSPVTMsg(const robognss::PVT &pvtMsg,
                      const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    const auto thisPVATime = rclcpp::Time(pvtMsg.header.stamp.sec, pvtMsg.header.stamp.nanosec, RCL_ROS_TIME);
    fgo::data::PVASolution pva;
    pva.wnc = pvtMsg.gnss_header.wnc;
    pva.tow = pvtMsg.gnss_header.tow;
    pva.sol_age = pvtMsg.gnss_header.sol_age;
    pva.diff_age = pvtMsg.gnss_header.diff_age;
    pva.timestamp = thisPVATime;
    pva.has_heading = false;
    pva.has_velocity = true;
    pva.has_roll_pitch = false;
    pva.error = pvtMsg.gnss_header.sol_status;
    pva.llh = (gtsam::Vector3() << pvtMsg.lat_deg * fgo::constants::deg2rad,
      pvtMsg.lon_deg * fgo::constants::deg2rad,
      pvtMsg.height + pva.undulation).finished();
    pva.xyz_ecef = fgo::utils::llh2xyz(pva.llh);
    const auto eRn = gtsam::Rot3(fgo::utils::nedRe_Matrix(pva.xyz_ecef)).inverse();
    pva.xyz_var = (gtsam::Vector3() << std::pow(pvtMsg.pos_std.x, 2),
      std::pow(pvtMsg.pos_std.y, 2),
      std::pow(pvtMsg.pos_std.z, 2)).finished();

    pva.nRe = gtsam::Rot3(utils::nedRe_Matrix_asLLH(pva.llh));

    pva.has_velocity = true;
    pva.has_velocity_3D = true;

    switch (pvtMsg.vel.vel_frame) {
      case robognss::Velocity::ECEF: {
        pva.vel_ecef = (gtsam::Vector3() << pvtMsg.vel.vx, pvtMsg.vel.vy, pvtMsg.vel.vz).finished();
        pva.vel_n = pva.nRe.rotate(pva.vel_ecef);
        pva.vel_var = (gtsam::Vector3() << std::pow(pvtMsg.vel.vx_std, 2),
          std::pow(pvtMsg.vel.vy_std, 2),
          std::pow(pvtMsg.vel.vz_std, 2)).finished();
        break;
      }
      case robognss::Velocity::NED: {
        pva.vel_n = (gtsam::Vector3() << pvtMsg.vel.vx, pvtMsg.vel.vy, pvtMsg.vel.vz).finished();
        pva.vel_ecef = pva.nRe.inverse().rotate(pva.vel_n);
        pva.vel_var = pva.nRe.inverse().rotate((gtsam::Vector3() << std::pow(pvtMsg.vel.vx_std, 2),
          std::pow(pvtMsg.vel.vy_std, 2),
          std::pow(pvtMsg.vel.vz_std, 2)).finished());
        break;
      }
      case robognss::Velocity::ENU: {
        pva.vel_n = (gtsam::Vector3() << pvtMsg.vel.vy, pvtMsg.vel.vx, -pvtMsg.vel.vz).finished();
        const auto enuRe = gtsam::Rot3(utils::enuRe_Matrix_asLLH(pva.llh));
        pva.vel_ecef = enuRe.inverse().rotate(pva.vel_n);
        pva.vel_var = enuRe.inverse().rotate((gtsam::Vector3() << std::pow(pvtMsg.vel.vy_std, 2),
          std::pow(pvtMsg.vel.vx_std, 2),
          std::pow(pvtMsg.vel.vz_std, 2)).finished());
        break;
      }
      case robognss::Velocity::BODY: {
        pva.vel_ecef = pva.rot_ecef.rotate((gtsam::Vector3() << pvtMsg.vel.vx, pvtMsg.vel.vy, pvtMsg.vel.vz).finished());
        pva.vel_n = pva.nRe.rotate(pva.vel_ecef);
        pva.vel_var = (gtsam::Vector3() << std::pow(pvtMsg.vel.vx_std, 2),
          std::pow(pvtMsg.vel.vy_std, 2),
          std::pow(pvtMsg.vel.vz_std, 2)).finished();
        pva.vel_var = pva.rot_ecef.rotate(pva.vel_var);
        break;
      }
    }

    pva.clock_bias = pvtMsg.rx_clk_bias;
    pva.clock_drift = pvtMsg.rx_clk_drift;
    pva.clock_bias_var = std::pow(pvtMsg.rx_clk_bias_std, 2);
    pva.clock_drift_var = std::pow(pvtMsg.rx_clk_drift_std, 2);

    pva.num_sat = pvtMsg.gnss_header.num_sat_all;
    pva.num_sat_used = pvtMsg.gnss_header.num_sat_used;
    pva.num_sat_used_l1 = pvtMsg.gnss_header.num_sat_l1e1;
    pva.num_sat_used_multi = pvtMsg.gnss_header.num_sat_multi;

    pva.type = fgo::utils::GNSS::getSBFSolutionType(pvtMsg.gnss_header.sol_status, pvtMsg.gnss_header.mode);
    return {pva, PVASolutionToState(pva, leverArm)};
  }


}


#endif //ONLINE_FGO_ROBOGNSSPARSER_H
