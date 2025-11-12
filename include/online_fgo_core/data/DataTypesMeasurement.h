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

#ifndef ONLINE_FGO_CORE_DATA_TYPES_MEASUREMENT_H
#define ONLINE_FGO_CORE_DATA_TYPES_MEASUREMENT_H

#pragma once

#include <atomic>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/NoiseModel.h>

#include "online_fgo_core/interface/TimeInterface.h"

// Note: Visual types (StereoPair, Image) are not included in core
// They will remain in the ROS adapter layers due to cv_bridge dependency

namespace fgo::data {

  /** IMU **/
  struct IMUMeasurement {
    fgo::core::TimeStamp timestamp = fgo::core::TimeStamp::zero();  // timestamp of current imu meas
    double dt{};  // dt between consequent meas
    gtsam::Rot3 AHRSOri{};
    gtsam::Matrix33 AHRSOriCov{};
    gtsam::Vector3 accLin{};
    gtsam::Matrix33 accLinCov{};
    gtsam::Vector3 accRot{};
    gtsam::Matrix33 accRotCov{};
    gtsam::Vector3 gyro{};
    gtsam::Matrix33 gyroCov{};
    gtsam::Vector3 mag{};
    gtsam::Matrix33 magCov{};
  };

  /** GNSS **/
  struct GNSSObs {
    uint32_t satId{};
    gtsam::Vector3 satPos{};
    gtsam::Vector3 satVel{};
    double pr{};
    double prVar{};
    double prVarRaw{};
    double dr{};
    double drVar{};
    double cp{};
    double cpVar{};
    double cpVarRaw{};
    double el{};
    double cn0{};
    double locktime{};
    bool cycleSlip = true;
    bool isLOS = true;
  };

  struct RefSat {
    uint8_t refSatSVID;
    gtsam::Vector3 refSatPos{};
    gtsam::Vector3 refSatVel{};
  };

  struct GNSSMeasurementEpoch {
    double tow{};
    fgo::core::TimeStamp timestamp = fgo::core::TimeStamp::zero();
    double delay;
    std::vector<GNSSObs> obs{};
    double timeOffsetGALGPS{};
    bool isGGTOValid = false;
    uint8_t integrityFlag{};
    gtsam::Vector3 basePosRTCM{};
    RefSat refSatGPS{};
    RefSat refSatGAL{};
    uint8_t ddIDXSyncRef;
    uint8_t ddIDXSyncUser;
  };

  enum GNSSSolutionType {
    RTKFIX = 1,
    RTKFLOAT = 2,
    SINGLE = 3,
    NO_SOLUTION = 4
  };

  struct PVASolution {
    uint16_t wnc;
    double tow{};
    fgo::core::TimeStamp timestamp = fgo::core::TimeStamp::zero();
    double delay = 0.;
    double sol_age = 0.;
    double diff_age = 0.;
    GNSSSolutionType type;
    uint8_t error;

    gtsam::Vector3 xyz_ecef;
    gtsam::Vector3 xyz_var;
    gtsam::Vector3 llh;
    gtsam::Vector3 vel_n;
    gtsam::Vector3 vel_ecef;
    gtsam::Vector3 vel_var;
    gtsam::Rot3 rot_n;
    gtsam::Rot3 rot_ecef;
    gtsam::Rot3 nRe;
    gtsam::Vector3 rot_var;
    double undulation;
    double heading;
    double cog;
    double heading_ecef;
    double heading_var;
    double roll_pitch;
    double roll_pitch_var;
    double trk_gnd;
    double clock_bias;
    double clock_drift;
    double clock_bias_var;
    double clock_drift_var;

    uint8_t num_sat;
    uint8_t num_sat_used;
    uint8_t num_sat_used_l1;
    uint8_t num_sat_used_multi;
    uint8_t num_bases;
    uint32_t reference_id;
    double correction_age;
    double solution_age;

    bool has_rotation_3D = false;
    bool has_heading;
    bool has_velocity;
    bool has_velocity_3D = false;
    bool has_roll_pitch;
  };

  struct GNSSMeasurement {
    bool hasRTK = false;
    GNSSMeasurementEpoch measMainAnt{};
    bool hasDualAntenna = false;
    GNSSMeasurementEpoch measAuxAnt{};
    bool hasDualAntennaDD = false;
    GNSSMeasurementEpoch measDualAntennaDD{};
    bool hasRTCMDD = false;
    GNSSMeasurementEpoch measRTCMDD{};
  };

  struct CycleSlipStruct {
    uint32_t satID{};
    int N{}; //count of not sliped
    double md{}; //mean
    double md2{}; //squared mean
    double sd2{}; //sigma
    bool connection{}; //can sat be found in newest gnss meas
  };

  struct CSDataStruct {
    uint32_t satID{};
    gtsam::Vector3 satPos{};
    double cp{};
    double cpVar{};
    double el{};
  };

  /** Timing **/
  struct PPS {
    std::atomic_uint_fast64_t counter;
    std::atomic_int_fast64_t localDelay;  // in milliseconds
    fgo::core::TimeStamp lastPPSTime;
  };

} // namespace fgo::data

#endif // ONLINE_FGO_CORE_DATA_TYPES_MEASUREMENT_H
