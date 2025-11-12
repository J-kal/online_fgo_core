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

#ifndef ONLINE_FGO_FACTORTYPEID_H
#define ONLINE_FGO_FACTORTYPEID_H
#pragma once

namespace fgo::factor {
enum FactorTypeID : unsigned int {
    CombinedIMU = 0,
    GPWNOJMotionPrior = 1,
    GPWNOAMotionPrior = 2,
    ReceiverClock = 3,
    NavPose = 4,
    GPNavPose = 5,
    NavAttitude = 6,
    GPNavAttitude = 7,
    NavVelocity = 8,
    GPNavVelocity = 9,
    BetweenPose = 10,
    GPDoubleBetweenPose = 11,
    GPSingleBetweenPose = 12,
    GPS = 13,
    GPGPS = 14,
    PVT = 15,
    GPPVT = 16,
    PRDR = 17,
    GPPRDR = 18,
    PR = 19,
    GPPR = 20,
    DR = 21,
    GPDR = 22,
    TDCP = 23,    // not used
    GPTDCP = 24,  // not used
    DDCP = 25,    // not used
    GPDDCP = 26,  // not used
    DDPR = 27,
    GPDDPR = 28,
    DDPRDR = 29,
    GPDDPRDR = 30,
    TDCP3 = 31,          // not used
    GPTDCP3 = 32,        // not used
    ConstVelocity = 33,  // not used
    ConstAngularVelocity = 34,
    ConstAcceleration = 35,
    GPSingerMotionPrior = 36,
    InvDepthProject = 37,
    GPInvDepthProject = 38,
    Project = 39,
    GPProject = 40,
    ProjectDepthOnly = 41,
    GPSingleProjectDepthOnly = 42,
    GPProjectDepthOnly = 43,
};

static const std::map<std::string, unsigned int> FactorNameIDMap = {
    {"CombinedIMUFactor", FactorTypeID::CombinedIMU},
    {"GPWNOAPriorPose3Factor", FactorTypeID::GPWNOAMotionPrior},
    {"GPWNOJPriorPose3Factor", FactorTypeID::GPWNOJMotionPrior},
    {"GPSingerPriorPose3Factor", FactorTypeID::GPSingerMotionPrior},
    {"ConstDriftFactor", FactorTypeID::ReceiverClock},
    {"NavPoseFactor", FactorTypeID::NavPose},
    {"GPInterpolatedNavPoseFactor", FactorTypeID::GPNavPose},
    {"NavAttitudeFactor", FactorTypeID::NavAttitude},
    {"GPInterpolatedNavAttitudeFactor", FactorTypeID::GPNavAttitude},
    {"NavVelocityFactor", FactorTypeID::NavVelocity},
    {"GPInterpolatedNavVelocityFactor", FactorTypeID::GPNavVelocity},
    {"BetweenPoseFactor", FactorTypeID::BetweenPose},
    {"GPInterpolatedDoublePose3BetweenFactor",
     FactorTypeID::GPDoubleBetweenPose},
    {"GPInterpolatedSinglePose3BetweenFactor",
     FactorTypeID::GPSingleBetweenPose},
    {"GPSFactor", FactorTypeID::GPS},
    {"GPInterpolatedGPSFactor", FactorTypeID::GPGPS},
    {"PVTFactor", FactorTypeID::PVT},
    {"GPInterpolatedPVTFactor", FactorTypeID::GPPVT},
    {"PrDrFactor", FactorTypeID::PRDR},
    {"GPInterpolatedPrDrFactor", FactorTypeID::GPPRDR},
    {"PrFactor", FactorTypeID::PR},
    {"GPInterpolatedPrFactor", FactorTypeID::GPPR},
    {"DrFactor", FactorTypeID::DR},
    {"GPInterpolatedDrFactor", FactorTypeID::GPDR},
    //{"NavAttitudeFactor", FactorTypeID::DDPR},
    // {"NavAttitudeFactor", FactorTypeID::GPDDPR},
    {"DDPrDrFactor", FactorTypeID::DDPRDR},
    {"GPInterpolatedDDPrDrFactor", FactorTypeID::GPDDPRDR},
    {"ConstAngularRateFactor", FactorTypeID::ConstAngularVelocity},
    {"ConstAccelerationFactor", FactorTypeID::ConstAcceleration},
    {"InverseDepthProjectFactor", FactorTypeID::InvDepthProject},
    {"GPInterpolatedInverseDepthProjectFactor",
     FactorTypeID::GPInvDepthProject},
    {"ProjectFactor", FactorTypeID::Project},
    {"GPProjectFactor", FactorTypeID::GPProject},
    {"ProjectDepthOnly", FactorTypeID::ProjectDepthOnly},
    {"GPSingleProjectDepthOnly", FactorTypeID::GPSingleProjectDepthOnly},
    {"GPProjectDepthOnly", FactorTypeID::GPProjectDepthOnly},
};

}  // namespace fgo::factor

#endif  // ONLINE_FGO_FACTORTYPEID_H

