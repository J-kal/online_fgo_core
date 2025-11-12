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


#ifndef ONLINE_FGO_GNSSTRANSUTILS_H
#define ONLINE_FGO_GNSSTRANSUTILS_H
#pragma once

#include <boost/optional.hpp>
#include <irt_nav_msgs/msg/gnss_obs_pre_processed.hpp>
#include <irt_nav_msgs/msg/pva_geodetic.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <ublox_msgs/msg/nav_clock.hpp>

#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <novatel_oem7_msgs/msg/bestvel.hpp>
#include <novatel_oem7_msgs/msg/dualantennaheading.hpp>
#include <novatel_oem7_msgs/msg/clockmodel.hpp>
#include <novatel_oem7_msgs/msg/inspvax.hpp>
#include <novatel_oem7_msgs/msg/inscov.hpp>

#include "data/DataTypesFGO.h"
#include "utils/NavigationTools.h"
#include "utils/GNSSUtils.h"
#include "integrator/param/IntegratorParams.h"
#include "utils/MeasurmentDelayCalculator.h"
#include "integrator/GNSSTCIntegrator.h"


namespace fgo::sensor::gnss {
  using namespace fgo::data;

  inline State PVASolutionToState(const PVASolution &pva,
                                  const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    fgo::data::State state;
    state.timestamp = pva.timestamp;
    state.state = gtsam::NavState(pva.rot_ecef, pva.xyz_ecef - pva.rot_ecef.rotate(leverArm), pva.vel_ecef);
    state.cbd = (gtsam::Vector2() << pva.clock_bias, pva.clock_drift).finished();
    return state;
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State>
  parseIRTPVAMsg(const irt_nav_msgs::msg::PVAGeodetic &pvaMsg,
                 const integrator::param::IntegratorBaseParamsPtr &paramPtr,
                 const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    const auto thisPVATime = rclcpp::Time(pvaMsg.header.stamp.sec, pvaMsg.header.stamp.nanosec, RCL_ROS_TIME);
    fgo::data::PVASolution pva;
    pva.tow = pvaMsg.tow;
    pva.timestamp = thisPVATime;
    pva.has_heading = true;
    pva.has_velocity = true;
    pva.has_roll_pitch = true;
    pva.error = pvaMsg.error;
    pva.llh = (gtsam::Vector3() << pvaMsg.phi_geo, pvaMsg.lambda_geo, pvaMsg.h_geo).finished();
    pva.xyz_ecef = fgo::utils::llh2xyz(pva.llh);
    const auto eRn = gtsam::Rot3(fgo::utils::nedRe_Matrix(pva.xyz_ecef)).inverse();
    pva.xyz_var = (gtsam::Vector3() << std::pow(pvaMsg.phi_geo_var, 2), std::pow(pvaMsg.lambda_geo_var, 2), std::pow(
      pvaMsg.h_geo_var, 2)).finished();
    //pva.xyz_var = eRn.rotate(pva.xyz_var);
    pva.vel_n = (gtsam::Vector3() << pvaMsg.vn, pvaMsg.ve, -pvaMsg.vu).finished();
    pva.vel_ecef = eRn.rotate(pva.vel_n);
    pva.vel_var = (gtsam::Vector3() << paramPtr->fixedVelVar, paramPtr->fixedVelVar, paramPtr->fixedVelVar).finished();
    pva.heading = pvaMsg.yaw * fgo::constants::deg2rad;
    pva.heading_var = std::pow(pvaMsg.yaw_var * fgo::constants::deg2rad, 2);

    if ((pvaMsg.yaw == 270. && pvaMsg.yaw_var == 0.) || pvaMsg.yaw_var > 10.) {
      pva.has_heading = false;
    } else {
      const auto heading_rot = gtsam::Rot3::Yaw(pva.heading);
      pva.heading_ecef = eRn.compose(heading_rot).yaw();
      pva.roll_pitch = pvaMsg.pitch_roll * fgo::constants::deg2rad;
      pva.roll_pitch_var = std::pow(pvaMsg.pitch_roll_var * fgo::constants::deg2rad, 2) * paramPtr->headingVarScale;

      if (paramPtr->hasRoll) {
        pva.rot_n = gtsam::Rot3::Ypr(pva.heading, 0., pva.roll_pitch);
        pva.rot_var = (gtsam::Vector3() << pva.roll_pitch_var, 0., pva.heading_var).finished();
      } else if (paramPtr->hasPitch) {
        pva.rot_n = gtsam::Rot3::Ypr(pva.heading, pva.roll_pitch, 0.);
        pva.rot_var = (gtsam::Vector3() << 0., pva.roll_pitch_var, pva.heading_var).finished();
      } else {
        pva.rot_n = gtsam::Rot3::Yaw(pva.heading);
        pva.rot_var = (gtsam::Vector3() << 0., 0., pva.heading_var).finished();
      }

      pva.rot_ecef = eRn.compose(pva.rot_n);
    }
    pva.type = fgo::utils::GNSS::getSBFSolutionType(pvaMsg.error, pvaMsg.mode);
    pva.num_sat = pvaMsg.nrsv_used_with_l1;

    return {pva, PVASolutionToState(pva, leverArm)};
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State>
  parseINSPVAXMsg(const novatel_oem7_msgs::msg::INSPVAX &pva,
                  const rclcpp::Time &msg_timestamp,
                  const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    fgo::data::PVASolution sol{};
    sol.timestamp = msg_timestamp;
    sol.tow = pva.nov_header.gps_week_milliseconds * 0.001;
    sol.type = fgo::utils::GNSS::getOEM7PVTSolutionType(pva.pos_type.type);
    sol.llh = (gtsam::Vector3() << pva.latitude * fgo::constants::deg2rad, pva.longitude * fgo::constants::deg2rad,
      pva.height + pva.undulation).finished();
    sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
    sol.xyz_var = (gtsam::Vector3() << pva.latitude_stdev, pva.longitude_stdev, pva.height_stdev).finished();
    sol.vel_n = (gtsam::Vector3() << pva.north_velocity, pva.east_velocity, -pva.up_velocity).finished();

    const auto eRned = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef)).inverse();
    sol.vel_ecef = eRned.rotate(sol.vel_n);
    sol.rot_n = gtsam::Rot3::Yaw(pva.azimuth * fgo::constants::deg2rad);

    sol.rot_ecef = eRned.compose(sol.rot_n);

    sol.rot_var = (gtsam::Vector3() << pva.azimuth_stdev * fgo::constants::deg2rad,
      pva.roll_stdev * fgo::constants::deg2rad,
      pva.pitch_stdev * fgo::constants::deg2rad).finished();
    sol.heading = -pva.azimuth * fgo::constants::deg2rad;
    sol.has_heading = true;
    sol.has_velocity = true;
    sol.has_roll_pitch = true;
    return {sol, PVASolutionToState(sol, leverArm)};
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State>
  parseNovAtelPVT(const novatel_oem7_msgs::msg::BESTPOS &bestpos,
                  const novatel_oem7_msgs::msg::BESTVEL &bestvel,
                  const rclcpp::Time &msg_timestamp,
                  const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    fgo::data::PVASolution sol{};
    sol.timestamp = msg_timestamp;
    sol.tow = bestpos.nov_header.gps_week_milliseconds * 0.001;
    sol.has_heading = false;
    sol.has_velocity = true;
    sol.llh = (gtsam::Vector3() << bestpos.lat * fgo::constants::deg2rad,
      bestpos.lon * fgo::constants::deg2rad,
      bestpos.hgt + bestpos.undulation).finished();
    sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
    sol.xyz_var = (gtsam::Vector3() << std::pow(bestpos.lat_stdev, 2), std::pow(bestpos.lon_stdev, 2), std::pow(
      bestpos.hgt_stdev, 2)).finished();
    const auto ecef_R_ned = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef).inverse());
    sol.vel_n = (gtsam::Vector3() << bestvel.hor_speed * std::cos(bestvel.trk_gnd * fgo::constants::deg2rad),
      bestvel.hor_speed * std::sin(bestvel.trk_gnd * fgo::constants::deg2rad),
      -bestvel.ver_speed).finished();
    sol.vel_ecef = ecef_R_ned.rotate(sol.vel_n);
    sol.type = fgo::utils::GNSS::getOEM7PVTSolutionType(bestpos.pos_type.type);
    sol.num_sat = bestpos.num_sol_svs;

    return {sol, PVASolutionToState(sol, leverArm)};
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State>
  parseNovAtelBestpos(const novatel_oem7_msgs::msg::BESTPOS &bestpos,
                      const rclcpp::Time &msg_timestamp,
                      const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    fgo::data::PVASolution sol{};
    sol.timestamp = msg_timestamp;
    sol.tow = bestpos.nov_header.gps_week_milliseconds * 0.001;
    sol.has_heading = false;
    sol.has_velocity = false;
    sol.llh = (gtsam::Vector3() << bestpos.lat * fgo::constants::deg2rad,
      bestpos.lon * fgo::constants::deg2rad,
      bestpos.hgt + bestpos.undulation).finished();
    sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
    sol.xyz_var = (gtsam::Vector3() << std::pow(bestpos.lat_stdev, 2), std::pow(bestpos.lon_stdev, 2), std::pow(
      bestpos.hgt_stdev, 2)).finished();

    //auto ecef_R_ned = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef).inverse());
    sol.type = fgo::utils::GNSS::getOEM7PVTSolutionType(bestpos.pos_type.type);
    sol.num_sat = bestpos.num_sol_svs;
    return {sol, PVASolutionToState(sol, leverArm)};
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State> parseUbloxPVTMsg(const ublox_msgs::msg::NavPVT &navpvt,
                                                                               const integrator::param::IntegratorBaseParamsPtr &paramPtr,
                                                                               const rclcpp::Time &msg_timestamp,
                                                                               const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    fgo::data::PVASolution sol{};
    sol.timestamp = msg_timestamp;
    sol.tow = navpvt.i_tow * 0.001;
    sol.has_velocity = true;
    sol.has_heading = true;
    sol.llh = (gtsam::Vector3() << navpvt.lat * 1e-7 * fgo::constants::deg2rad,
      navpvt.lon * 1e-7 * fgo::constants::deg2rad,
      navpvt.height * 1e-3).finished();
    sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
    sol.xyz_var = (gtsam::Vector3() << std::pow(navpvt.h_acc * 1e-3, 2), std::pow(navpvt.h_acc * 1e-3, 2),
      std::pow(navpvt.v_acc * 1e-3, 2) * 2).finished();
    sol.vel_n = (gtsam::Vector3() << navpvt.vel_n * 1e-3, navpvt.vel_e * 1e-3, navpvt.vel_d * 1e-3).finished();
    const auto ned_R_e = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef));
    const auto ecef_R_ned = ned_R_e.inverse();

    //sol.xyz_var = ecef_R_ned.rotate(sol.xyz_var);
    sol.vel_ecef = ecef_R_ned.rotate(sol.vel_n);
    const auto velVar = std::pow(navpvt.s_acc * 1e-3, 2);
    sol.vel_var = (gtsam::Vector3() << velVar, velVar, velVar).finished();
    //sol.vel_var = ecef_R_ned.rotate(sol.vel_var);
    const auto heading = navpvt.heading * 1e-5 + paramPtr->heading_offset_deg;
    if (heading > 360.)
      sol.heading = (heading - 360.) * fgo::constants::deg2rad;
    else
      sol.heading = heading * fgo::constants::deg2rad;
    sol.rot_n = gtsam::Rot3::Yaw(sol.heading);
    sol.rot_ecef = ecef_R_ned.compose(sol.rot_n);
    sol.heading_ecef = sol.rot_ecef.yaw();
    sol.heading_var = std::pow(navpvt.head_acc * 1e-5 * fgo::constants::deg2rad, 2);
    sol.rot_var = (gtsam::Vector3() << 0, 0., sol.heading_var).finished();
    sol.type = fgo::utils::GNSS::getUbloxSolutionType(navpvt.fix_type, navpvt.flags);
    sol.num_sat = navpvt.num_sv;

    return {sol, PVASolutionToState(sol, leverArm)};
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State> parseNavFixMsg(const sensor_msgs::msg::NavSatFix &msg,
                                                                             const rclcpp::Time &msg_timestamp,
                                                                             const gtsam::Vector3 &leverArm = gtsam::Vector3()) {
    fgo::data::PVASolution sol{};
    sol.timestamp = msg_timestamp;
    sol.llh = (gtsam::Vector3() << msg.latitude * fgo::constants::deg2rad,
      msg.longitude * fgo::constants::deg2rad,
      msg.altitude).finished();
    sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
    sol.type = data::GNSSSolutionType::RTKFIX;
    return {sol, PVASolutionToState(sol, leverArm)};
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State> parseNavFixWithTwist(const sensor_msgs::msg::NavSatFix &fix,
                                                                                   const geometry_msgs::msg::TwistStamped &twist,
                                                                                   const rclcpp::Time &msg_timestamp,
                                                                                   const std::vector<IMUMeasurement> &imu,
                                                                                   const gtsam::Vector3 &leverArm = gtsam::Vector3(),
                                                                                   const bool isENU=true) {
    fgo::data::PVASolution sol{};
    sol.timestamp = msg_timestamp;
    sol.llh = (gtsam::Vector3() << fix.latitude * fgo::constants::deg2rad,
      fix.longitude * fgo::constants::deg2rad,
      fix.altitude).finished();
    sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
    sol.type = data::GNSSSolutionType::RTKFIX;
    sol.xyz_var = (gtsam::Vector3() << 0.05,
      0.05,
      0.05).finished();
    sol.has_velocity = true;

    if(isENU)
    {
      sol.nRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(sol.xyz_ecef));
    }
    else
    {
      sol.nRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef));
    }

    if(!imu.empty()) {
      const auto& last_imu = imu.back();
      sol.rot_n = last_imu.AHRSOri;
      sol.rot_ecef = sol.nRe.inverse().compose(sol.rot_n);
      sol.rot_var = (gtsam::Vector3() << 0.05,
           0.05,
           0.05).finished();

      sol.vel_ecef = sol.rot_ecef.rotate((gtsam::Vector3() << twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z).finished());
      sol.vel_n = sol.nRe.rotate(sol.vel_ecef);
      sol.vel_var = (gtsam::Vector3() << 0.1,
        0.1,
        0.1).finished();
    }
    else {
      sol.rot_var = (gtsam::Vector3() << 3.1415*3.1415,
     3.1415*3.1415,
     3.1415*3.1415).finished();
      sol.vel_var = (gtsam::Vector3() << 100,
       100,
       100).finished();
    }
    return {sol, PVASolutionToState(sol, leverArm)};
  }

  inline std::tuple<fgo::data::PVASolution, fgo::data::State> parseOdomMsg(const nav_msgs::msg::Odometry &pva,
                                                                           const rclcpp::Time &msg_timestamp,
                                                                           const gtsam::Vector3 &leverArm = gtsam::Vector3(),
                                                                           const gtsam::Rot3 &imuPreRotation = gtsam::Rot3()) {
    // ToDo: This function is only used for boreas dataset currently!
    fgo::data::PVASolution this_pva{};
    this_pva.timestamp = msg_timestamp;
    this_pva.tow = msg_timestamp.seconds();
    this_pva.type = fgo::data::GNSSSolutionType::RTKFIX;

    this_pva.llh = (gtsam::Vector3() << pva.pose.pose.position.x * fgo::constants::deg2rad,
      pva.pose.pose.position.y * fgo::constants::deg2rad,
      pva.pose.pose.position.z).finished();

    /* !
     *  TODO: variance for the pva: is this provided in the odometry msg?
     */

    this_pva.xyz_ecef = fgo::utils::llh2xyz(this_pva.llh);
    this_pva.xyz_var = (gtsam::Vector3() << .05, 0.05, .05).finished();
    //const auto nedRenu = gtsam::Rot3(fgo::utils::nedRenu_llh(this_pva.llh));
    const auto eRenu = gtsam::Rot3(fgo::utils::enuRe_Matrix_asLLH(this_pva.llh)).inverse();
    const auto nedRe = gtsam::Rot3(fgo::utils::nedRe_Matrix_asLLH(this_pva.llh));
    const auto eRned = nedRe.inverse();
    //const auto nedRenu = nedRe * eRenu;
    const auto vel_ned = (gtsam::Vector3() << pva.twist.twist.linear.y,
      pva.twist.twist.linear.x,
      -pva.twist.twist.linear.z).finished();

    const auto rot_enu = gtsam::Rot3::Quaternion(pva.pose.pose.orientation.w,
                                                 pva.pose.pose.orientation.x,
                                                 pva.pose.pose.orientation.y,
                                                 pva.pose.pose.orientation.z);

    const auto vel_body_applanix = rot_enu.inverse().rotate(vel_ned);

    this_pva.vel_n = vel_ned;
    this_pva.vel_ecef = this_pva.rot_ecef.rotate(vel_body_applanix);


    const auto rot_ned_imu = rot_enu.compose(imuPreRotation.inverse());

    this_pva.rot_n = rot_enu;

    this_pva.rot_ecef = eRned.compose(this_pva.rot_n);
    this_pva.has_heading = true;
    this_pva.heading = this_pva.rot_n.yaw() * fgo::constants::rad2deg;
    this_pva.has_roll_pitch = true;
    this_pva.has_velocity_3D = true;
    this_pva.has_velocity = true;
    return {this_pva, PVASolutionToState(this_pva, leverArm)};
  }

  inline void extractGNSSObs(const irt_nav_msgs::msg::GNSSObs &gnssObsMsg,
                             std::vector<fgo::data::GNSSObs> &gnssObsVec,
                             const integrator::param::IntegratorGNSSTCParamsPtr &paramPtr,
                             bool isMainAnt = true,
                             boost::optional<std::vector<irt_nav_msgs::msg::SatLabel> &> satLabels = boost::none,
                             boost::optional<std::map<int, bool> &> LOSLoopUp = boost::none) {
    //RCLCPP_INFO(this->get_logger(), "/////////////////////////////////////////////");
    //std::list<int> alreadyin;
    for (size_t i = 0; i < gnssObsMsg.prn.size(); i++) {
      double pr = gnssObsMsg.pseudorange[i];
      //when pr = 0 or nan then do nothing
      if (pr == 0.0 || (pr != pr))
        continue;

      fgo::data::GNSSObs gnssObs;
      //RCLCPP_INFO_STREAM(this->get_logger(), "PRN: " << gnssObsMsg.prn[i]);
      gnssObs.satId = gnssObsMsg.prn[i];
      //RCLCPP_INFO_STREAM(this->get_logger(), "SatPos: x: " << std::fixed << gnssObsMsg.satelite_pos[i].x << " y: " <<  gnssObsMsg.satelite_pos[i].y << " z: " << gnssObsMsg.satelite_pos[i].z);
      gnssObs.satPos = gtsam::Vector3(gnssObsMsg.satelite_pos[i].x,
                                      gnssObsMsg.satelite_pos[i].y,
                                      gnssObsMsg.satelite_pos[i].z);
      //RCLCPP_INFO_STREAM(this->get_logger(), "SatPos: "<< std::fixed  <<  gnssObs.satPos.transpose());
      //RCLCPP_INFO_STREAM(this->get_logger(), "POSNORM: "<< std::fixed  <<  gnssObs.satPos.norm());
      gnssObs.satVel = gtsam::Vector3(gnssObsMsg.satelite_vec[i].x,
                                      gnssObsMsg.satelite_vec[i].y,
                                      gnssObsMsg.satelite_vec[i].z);
      //RCLCPP_INFO_STREAM(this->get_logger(), "SatVelNorm: " << std::fixed  <<  gnssObs.satVel.norm());
      //RCLCPP_INFO_STREAM(this->get_logger(), "PR: " << std::fixed  <<  gnssObsMsg.pseudorange[i]);
      gnssObs.pr = gnssObsMsg.pseudorange[i];
      //RCLCPP_INFO_STREAM(this->get_logger(), "PR: " << std::fixed  <<  gnssObs.pr);

      if (isMainAnt) {
        gnssObs.drVar = gnssObsMsg.deltarange_var[i] * pow(paramPtr->dopplerrangeVarScaleAntMain, 2);
        if (!paramPtr->pseudorangeUseRawStd) {
          gnssObs.prVar = gnssObsMsg.pseudorange_var[i] * pow(paramPtr->pseudorangeVarScaleAntMain, 2);
        } else {
          gnssObs.prVar = pow(gnssObsMsg.pseudorange_var_measured[i], 2) * paramPtr->pseudorangeVarScaleAntMain;
        }

        if (LOSLoopUp) {
          auto LOSIter = LOSLoopUp->find(gnssObs.satId);
          if (LOSIter != LOSLoopUp->end()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("gnss_fgo"),
                                "PRN " << gnssObs.satId << " is LOS? " << LOSIter->second);
            gnssObs.isLOS = LOSIter->second;
          } else
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("gnss_fgo"), "PRN " << gnssObs.satId << " NOT IN LOOPUP Table");

        }

      } else {
        gnssObs.drVar = gnssObsMsg.deltarange_var[i] * pow(paramPtr->dopplerrangeVarScaleAntAux, 2);
        if (!paramPtr->pseudorangeUseRawStd) {
          gnssObs.prVar = gnssObsMsg.pseudorange_var[i] * pow(paramPtr->pseudorangeVarScaleAntAux, 2);
        } else {
          gnssObs.prVar = pow(gnssObsMsg.pseudorange_var_measured[i], 2) * paramPtr->pseudorangeVarScaleAntAux;
        }
      }
      gnssObs.prVarRaw = std::pow(gnssObsMsg.pseudorange_var_measured[i], 2);
      gnssObs.dr = gnssObsMsg.deltarange[i];
      //RCLCPP_INFO_STREAM(this->get_logger(), "DR: " << std::fixed  <<  gnssObs.dr);

      gnssObs.cp = gnssObsMsg.carrierphase[i];
      gnssObs.el = gnssObsMsg.elevation_angle[i]; //fgo::utils::calcEl(gnssObs.satPos, currentPredState_.state.t());
      gnssObs.cn0 = gnssObsMsg.cn0[i];
      gnssObs.cpVar = fgo::utils::GNSS::calculateDDVariance(paramPtr->weightingModel, paramPtr->carrierphaseStd,
                                                            paramPtr->lambdaL1, //TODO @Haoming how to fix?
                                                            gnssObs.el, gnssObsMsg.carrierphase_var_measured[i], "");
      gnssObs.cpVarRaw =
        std::pow(gnssObsMsg.carrierphase_var_measured[i] * paramPtr->lambdaL1, 2) * paramPtr->carrierStdScale;
      gnssObs.locktime = gnssObsMsg.locktime[i];

      if (satLabels) {
        irt_nav_msgs::msg::SatLabel label;
        label.prn = gnssObs.satId;
        label.sat_pos = fgo::utils::convertGTVec3ToROS(gnssObs.satPos);
        label.sat_vel = fgo::utils::convertGTVec3ToROS(gnssObs.satVel);
        label.psr = gnssObs.pr;
        label.psr_raw = gnssObsMsg.pseudorange_raw[i];
        label.psr_satclk_corrected = gnssObsMsg.pseudorange_satclk_corrected[i];
        label.psr_dev_preproc = gnssObsMsg.pseudorange_var[i];
        label.psr_dev_measured = gnssObsMsg.pseudorange_var_measured[i];
        label.dr = gnssObs.dr;
        label.dr_raw = gnssObsMsg.deltarange_raw[i];
        label.dr_satclk_corrected = gnssObsMsg.deltarange_satclk_corrected[i];
        label.dr_dev_preproc = gnssObsMsg.deltarange_var[i];
        label.cp = gnssObs.cp;
        label.cp_raw = gnssObsMsg.carrierphase_raw[i];
        label.cp_satclk_corrected = gnssObsMsg.carrierphase_satclk_corrected[i];
        label.cp_dev_measured = gnssObsMsg.carrierphase_var_measured[i];
        label.locktime = gnssObs.locktime;
        label.cn0 = gnssObsMsg.cn0[i];
        label.elevation_angle = gnssObs.el;
        label.azimuth_angle = gnssObsMsg.azimuth_angle[i];
        satLabels->emplace_back(label);
      }
      gnssObsVec.emplace_back(gnssObs);
    }
  }

  inline fgo::data::GNSSMeasurement convertGNSSObservationMsg(const irt_nav_msgs::msg::GNSSObsPreProcessed &gnssMsg,
                                                              const integrator::param::IntegratorGNSSTCParamsPtr &paramPtr,
                                                              boost::optional<irt_nav_msgs::msg::GNSSLabeling &> satLabel = boost::none) {
    static std::map<uint32_t, double> PRNLocktimeMapAntMain = fgo::utils::GNSS::GetInitPRNLocktimeMap();
    static std::map<uint32_t, double> PRNLocktimeMapAntAux = fgo::utils::GNSS::GetInitPRNLocktimeMap();
    static std::vector<uint32_t> lastPRNAntMain;
    static std::vector<uint32_t> lastPRNAntAux;
    static std::vector<fgo::data::CycleSlipStruct> cycleSlipAux;
    static std::vector<fgo::data::CycleSlipStruct> cycleSlipRTCM;
    static std::vector<fgo::data::CycleSlipStruct> cycleSlipMain;
    static uint lastRTCMRefSatID;
    static uint lastAuxRefSatID;
    static uint lastMainRefSatID;

    const auto thisGNSSTime = rclcpp::Time(gnssMsg.header.stamp.sec, gnssMsg.header.stamp.nanosec, RCL_ROS_TIME);
    fgo::data::GNSSMeasurement gnssMeas;

    gnssMeas.measMainAnt.tow = gnssMsg.gnss_obs_ant_main.time_receive;
    gnssMeas.measMainAnt.timestamp = thisGNSSTime;

    gnssMeas.measMainAnt.timeOffsetGALGPS = gnssMsg.time_offset_gal_gps;
    gnssMeas.measMainAnt.isGGTOValid = gnssMsg.is_ggto_valid;
    gnssMeas.measMainAnt.integrityFlag = gnssMsg.gnss_obs_ant_main.integrity_flag;

    if (satLabel) {
      extractGNSSObs(gnssMsg.gnss_obs_ant_main, gnssMeas.measMainAnt.obs, paramPtr, true, satLabel->ant_main_labels);
      fgo::utils::GNSS::getRTKCorrectionsFromROSMsg(gnssMsg.gnss_corrections, satLabel->ant_main_labels);
      for (const auto &prn: gnssMsg.faulty_prn_main) {
        if (prn)
          satLabel->faulty_prn_main.emplace_back(prn);
      }
    } else {
      extractGNSSObs(gnssMsg.gnss_obs_ant_main, gnssMeas.measMainAnt.obs, paramPtr);
    }

    fgo::utils::GNSS::checkCycleSlipByLocktime(gnssMeas.measMainAnt, PRNLocktimeMapAntMain, lastPRNAntMain);
    if (satLabel) {
      for (const auto &obs: gnssMeas.measMainAnt.obs) {
        for (auto &label: satLabel->ant_main_labels) {
          if (label.prn == obs.satId) {
            label.cycle_slip = obs.cycleSlip;
            break;
          }
        }
      }
    }
    // RCLCPP_WARN_STREAM(this->get_logger(), "------- Main Ant LockTime Start: --------");
    //for(const auto& obs : gnssMeas.measMainAnt.obs)
    //{
    //   std::cout << "PRN: " << obs.satId << " locktime: " << obs.locktime << " cycleSlip? " << obs.cycleSlip << std::endl;
    //}
    //RCLCPP_WARN_STREAM(this->get_logger(), "------- Main Ant LockTime End: --------");

    //for (auto &x : gnssMeas.measMainAnt.obs){
    //  x.prVar *= 1;
    //  x.drVar *= 1;
    // }

    //second antenna
    if (gnssMsg.has_dualantenna) {
      //RCLCPP_INFO(this->get_logger(), "GNSSMsgAux");
      gnssMeas.hasDualAntenna = true;
      gnssMeas.measAuxAnt.tow = gnssMsg.gnss_obs_ant_aux.time_receive;
      gnssMeas.measAuxAnt.timestamp = thisGNSSTime;
      gnssMeas.measAuxAnt.timeOffsetGALGPS = gnssMsg.time_offset_gal_gps;
      gnssMeas.measAuxAnt.isGGTOValid = gnssMsg.is_ggto_valid;
      gnssMeas.measAuxAnt.integrityFlag = gnssMsg.gnss_obs_ant_aux.integrity_flag;


      if (satLabel) {
        extractGNSSObs(gnssMsg.gnss_obs_ant_aux, gnssMeas.measAuxAnt.obs, paramPtr, false, satLabel->ant_aux_labels);
        fgo::utils::GNSS::getRTKCorrectionsFromROSMsg(gnssMsg.gnss_corrections, satLabel->ant_aux_labels);


        for (const auto &prn: gnssMsg.faulty_prn_aux) {
          if (prn)
            satLabel->faulty_prn_aux.emplace_back(prn);
        }
      } else
        extractGNSSObs(gnssMsg.gnss_obs_ant_aux, gnssMeas.measAuxAnt.obs, paramPtr, false);

      fgo::utils::GNSS::checkCycleSlipByLocktime(gnssMeas.measAuxAnt, PRNLocktimeMapAntAux, lastPRNAntAux);
      if (satLabel) {
        for (const auto &obs: gnssMeas.measAuxAnt.obs) {
          for (auto &label: satLabel->ant_aux_labels) {
            if (label.prn == obs.satId) {
              label.cycle_slip = obs.cycleSlip;
              break;
            }
          }
        }
      }
    }

    if (gnssMsg.has_dualantenna_dd && paramPtr->useDualAntennaDD) {
      //RCLCPP_INFO(this->get_logger(), "GNSSMsgDUALDD");
      gnssMeas.hasDualAntennaDD = true;
      gnssMeas.measDualAntennaDD.tow = gnssMsg.dd_gnss_obs_dualantenna.time_receive;
      gnssMeas.measDualAntennaDD.timestamp = thisGNSSTime;
      gnssMeas.measDualAntennaDD.timeOffsetGALGPS = gnssMsg.time_offset_gal_gps;
      gnssMeas.measDualAntennaDD.isGGTOValid = gnssMsg.is_ggto_valid;
      gnssMeas.measDualAntennaDD.integrityFlag = gnssMsg.dd_gnss_obs_dualantenna.integrity_flag;
      gnssMeas.measDualAntennaDD.refSatGPS.refSatSVID = gnssMsg.dd_gnss_obs_dualantenna.ref_sat_svid_gps;
      gnssMeas.measDualAntennaDD.refSatGAL.refSatSVID = gnssMsg.dd_gnss_obs_dualantenna.ref_sat_svid_gal;
      gnssMeas.measDualAntennaDD.ddIDXSyncRef = gnssMsg.dd_gnss_obs_dualantenna.dd_idx_sync_ref;
      gnssMeas.measDualAntennaDD.ddIDXSyncUser = gnssMsg.dd_gnss_obs_dualantenna.dd_idx_sync_user;
      uint refSatID = 255;
      for (size_t i = 0; i < gnssMsg.dd_gnss_obs_dualantenna.prn.size(); i++) {
        if (gnssMsg.dd_gnss_obs_dualantenna.prn[i] == gnssMsg.dd_gnss_obs_dualantenna.ref_sat_svid_gps) {
          refSatID = i;
          break;
        }
      }
      if (refSatID != 255) {
        //RCLCPP_INFO_STREAM(this->get_logger(), "12: " << refSatID);
        gnssMeas.measDualAntennaDD.refSatGPS.refSatPos = gtsam::Vector3(
          gnssMsg.dd_gnss_obs_dualantenna.satelite_pos[refSatID].x,
          gnssMsg.dd_gnss_obs_dualantenna.satelite_pos[refSatID].y,
          gnssMsg.dd_gnss_obs_dualantenna.satelite_pos[refSatID].z);
        gnssMeas.measDualAntennaDD.refSatGPS.refSatVel = gtsam::Vector3(
          gnssMsg.dd_gnss_obs_dualantenna.satelite_vec[refSatID].x,
          gnssMsg.dd_gnss_obs_dualantenna.satelite_vec[refSatID].y,
          gnssMsg.dd_gnss_obs_dualantenna.satelite_vec[refSatID].z);
      }
      refSatID = 255;
      for (size_t i = 0; i < gnssMsg.dd_gnss_obs_dualantenna.prn.size(); i++) {
        if (gnssMsg.dd_gnss_obs_dualantenna.prn[i] == gnssMsg.dd_gnss_obs_dualantenna.ref_sat_svid_gal) {
          refSatID = i;
          break;
        }
      }
      if (refSatID != 255) {
        gnssMeas.measDualAntennaDD.refSatGAL.refSatPos = gtsam::Vector3(
          gnssMsg.dd_gnss_obs_dualantenna.satelite_pos[refSatID].x,
          gnssMsg.dd_gnss_obs_dualantenna.satelite_pos[refSatID].y,
          gnssMsg.dd_gnss_obs_dualantenna.satelite_pos[refSatID].z);
        gnssMeas.measDualAntennaDD.refSatGAL.refSatVel = gtsam::Vector3(
          gnssMsg.dd_gnss_obs_dualantenna.satelite_vec[refSatID].x,
          gnssMsg.dd_gnss_obs_dualantenna.satelite_vec[refSatID].y,
          gnssMsg.dd_gnss_obs_dualantenna.satelite_vec[refSatID].z);
      }

      extractGNSSObs(gnssMsg.dd_gnss_obs_dualantenna, gnssMeas.measDualAntennaDD.obs, paramPtr);
      fgo::utils::GNSS::checkCycleSlip(gnssMeas.measDualAntennaDD, lastAuxRefSatID, cycleSlipAux, false);
    }

    gnssMeas.hasRTK = gnssMsg.has_rtk;

    if (gnssMsg.has_rtcm_dd &&
        paramPtr->useRTCMDD) { // if gnssMeas.measRTCMDD.tow is zero, then we don't have this && gnssMsg->dd_gnss_obs_rtcm.time_receive != 0.
      //RCLCPP_INFO(this->get_logger(), "GNSSMsgRTCMDD");
      gnssMeas.hasRTCMDD = true;
      gnssMeas.measRTCMDD.tow = gnssMsg.dd_gnss_obs_rtcm.time_receive;
      gnssMeas.measRTCMDD.timestamp = thisGNSSTime;
      gnssMeas.measRTCMDD.timeOffsetGALGPS = gnssMsg.time_offset_gal_gps;
      gnssMeas.measRTCMDD.isGGTOValid = gnssMsg.is_ggto_valid;
      gnssMeas.measRTCMDD.integrityFlag = gnssMsg.dd_gnss_obs_rtcm.integrity_flag;

      gnssMeas.measRTCMDD.basePosRTCM = gtsam::Vector3(gnssMsg.dd_gnss_obs_rtcm.base_pos.x,
                                                       gnssMsg.dd_gnss_obs_rtcm.base_pos.y,
                                                       gnssMsg.dd_gnss_obs_rtcm.base_pos.z);
      //refsat
      gnssMeas.measRTCMDD.refSatGPS.refSatSVID = gnssMsg.dd_gnss_obs_rtcm.ref_sat_svid_gps;
      //RCLCPP_INFO_STREAM(this->get_logger(), "refSatID: " << unsigned(gnssMsg->dd_gnss_obs_rtcm.ref_sat_svid_gps));
      gnssMeas.measRTCMDD.refSatGAL.refSatSVID = gnssMsg.dd_gnss_obs_rtcm.ref_sat_svid_gal;
      gnssMeas.measRTCMDD.ddIDXSyncRef = gnssMsg.dd_gnss_obs_rtcm.dd_idx_sync_ref;
      gnssMeas.measRTCMDD.ddIDXSyncUser = gnssMsg.dd_gnss_obs_rtcm.dd_idx_sync_user;
      uint refSatID = 255;
      for (size_t i = 0; i < gnssMsg.dd_gnss_obs_rtcm.prn.size(); i++) {
        if (gnssMsg.dd_gnss_obs_rtcm.prn[i] == gnssMsg.dd_gnss_obs_rtcm.ref_sat_svid_gps) {
          refSatID = i;
          break;
        }
      }
      if (refSatID != 255) {
        gnssMeas.measRTCMDD.refSatGPS.refSatPos = gtsam::Vector3(
          gnssMsg.dd_gnss_obs_rtcm.satelite_pos[refSatID].x,
          gnssMsg.dd_gnss_obs_rtcm.satelite_pos[refSatID].y,
          gnssMsg.dd_gnss_obs_rtcm.satelite_pos[refSatID].z);
        gnssMeas.measRTCMDD.refSatGPS.refSatVel = gtsam::Vector3(
          gnssMsg.dd_gnss_obs_rtcm.satelite_vec[refSatID].x,
          gnssMsg.dd_gnss_obs_rtcm.satelite_vec[refSatID].y,
          gnssMsg.dd_gnss_obs_rtcm.satelite_vec[refSatID].z);
      }
      refSatID = 255;
      for (size_t i = 0; i < gnssMsg.dd_gnss_obs_rtcm.prn.size(); i++) {
        if (gnssMsg.dd_gnss_obs_rtcm.prn[i] == gnssMsg.dd_gnss_obs_rtcm.ref_sat_svid_gal) {
          refSatID = i;
          break;
        }
      }
      if (refSatID != 255) {
        gnssMeas.measRTCMDD.refSatGAL.refSatPos = gtsam::Vector3(
          gnssMsg.dd_gnss_obs_rtcm.satelite_pos[refSatID].x,
          gnssMsg.dd_gnss_obs_rtcm.satelite_pos[refSatID].y,
          gnssMsg.dd_gnss_obs_rtcm.satelite_pos[refSatID].z);
        gnssMeas.measRTCMDD.refSatGAL.refSatVel = gtsam::Vector3(
          gnssMsg.dd_gnss_obs_rtcm.satelite_vec[refSatID].x,
          gnssMsg.dd_gnss_obs_rtcm.satelite_vec[refSatID].y,
          gnssMsg.dd_gnss_obs_rtcm.satelite_vec[refSatID].z);
      }
      extractGNSSObs(gnssMsg.dd_gnss_obs_rtcm, gnssMeas.measRTCMDD.obs, paramPtr);

      if (gnssMeas.measRTCMDD.obs.empty())
        gnssMeas.hasRTCMDD = false;

      if (gnssMeas.hasRTCMDD) {
        RCLCPP_INFO(rclcpp::get_logger("gnss_fgo"), "CycleSlip Start..");
        fgo::utils::GNSS::checkCycleSlip(gnssMeas.measRTCMDD, lastRTCMRefSatID, cycleSlipRTCM, false);
        for (fgo::data::CycleSlipStruct &x: cycleSlipRTCM) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("gnss_fgo"),
                             "satID: " << x.satID << " md: " << x.md << " md2: " << x.md2 << " sd2: " << x.sd2 << " N:"
                                       << x.N);
        }
      }
    }
    return gnssMeas;
  }


}
#endif //ONLINE_FGO_GNSSTRANSUTILS_H
