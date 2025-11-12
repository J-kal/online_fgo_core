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
// Created by haoming on 08.05.24.
//

#ifndef ONLINE_FGO_BEACON_H
#define ONLINE_FGO_BEACON_H
#ifdef ENABLE_BEACON

#include <beacon/core/timestamp.h>
#include <beacon_module/image_drawer/image_drawer.h>
#include <beacon_module/odometry/stereo_odometry.h>
// #include <beacon_module/stereo_odometry/stereo_odometry.h>

#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "integrator/param/IntegratorParams.h"
#include "model/gp_interpolator/GPInterpolatorBase.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "solver/FixedLagSmoother.h"

namespace sensors::Camera {

struct BeaconStereoPairInput {
    cv::Mat left;
    beacon::Timestamp left_timestamp;
    cv::Mat right;
    beacon::Timestamp right_timestamp;

};  // struct BeaconStereoPairInput

struct BeaconStereoPair {
    beacon::ImageConstPtr left{};
    beacon::ImageConstPtr right{};
    beacon::Timestamp timestamp;

};  // struct BeaconStereoPair

class BeaconStereoPipeline {
    using ImagePublisher =
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;
    using NavPublisher =
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr;

    bool initialized_{};
    bool get_first_data_{};

    rclcpp::Node& node_;

    // ----------------------- Field for querying the state --------------------
    fgo::integrator::param::IntegratorVisualParamsPtr integratorParamPtr_{};
    std::shared_ptr<fgo::models::GPInterpolator> interpolator_{};

    mutable std::mutex optimize_state_mtx_{};
    std::map<size_t, std::pair<rclcpp::Time, fgo::data::QueryStateInput>>
        optimized_state_map_{};  // optPoseIndexPairMap_
    fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap
        optimize_input_state_map_{};  // currentOptPoseIndexTimestampMap_

    // ------------------------------ publisher fields ------------------------
    ImagePublisher left_img_pub_{};    // left image with feature
    ImagePublisher right_img_pub_{};   // right image with feature
    ImagePublisher mono_img_pub_{};    // image with monocular match result
    ImagePublisher stereo_img_pub_{};  // image with stereo match result
    NavPublisher base_pub{};

    // ------------------------------ beacon field ----------------------------
    beacon::StereoOdometryPtr odom_{};

    beacon::CameraPtr left_cam_{};
    gtsam::Pose3 T_bl{};  // left cam in base

    beacon::CameraPtr right_cam_{};
    gtsam::Pose3 T_br{};
    // gtsam::Pose3 T_er{};

    beacon::BeaconLoggerPtr beacon_logger_{};
    beacon::ImageDrawer left_drawer_{};
    beacon::ImageDrawer right_drawer_{};

    std::jthread process_thread_{};

    mutable std::mutex input_buffer_mtx_{};
    std::condition_variable input_buffer_cv_{};
    std::deque<BeaconStereoPairInput> input_buffer_{};

    std::deque<BeaconStereoPair> keyframe_buffer_{};

    // ------------------------------ anchor ----------------------------------
    // anchor point buffer
    // anchor point got from first frame
    // w -> anchor body frame, local world
    // e -> ecef
    // n -> navigation, enu
    gtsam::Pose3 anchor_T_ew{};
    gtsam::Point3 anchor_t_ew{};
    gtsam::Pose3 anchor_T_we{};
    gtsam::Rot3 anchor_R_ne{};
    gtsam::Rot3 anchor_R_wn{};
    double anchor_yaw_nw{};

  public:
    explicit BeaconStereoPipeline(
        rclcpp::Node& node,
        fgo::integrator::param::IntegratorVisualParamsPtr integratorParamPtr);

    void process(cv::Mat left_img,
                 const beacon::Timestamp& left_timestamp,
                 cv::Mat right_img,
                 const beacon::Timestamp& right_timestamp);

    void loadBeaconParameter(const beacon::ParameterHandler& param);

    bool init();

  protected:
    void processImpl();

    beacon::ImagePtr preprocessImage(cv::Mat img,
                                     const beacon::Timestamp& time,
                                     const beacon::CameraPtr& cam,
                                     const beacon::Transform& T_wb);

  public:
    // fgo buffer helper
    void updateOptPose(size_t id,
                       const rclcpp::Time& timestamp,
                       const fgo::data::QueryStateInput& input);

    void updateKeyIndexTimestampMap(
        const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap&
            currentKeyIndexTimestampMap);

  protected:
    fgo::data::QueryStateOutput queryBasePoseFromOptPose(
        const rclcpp::Time& timestamp);

};  // class Beacon

}  // namespace sensors::Camera

#endif  // ENABLE_BEACON

#endif  // ONLINE_FGO_BEACON_H
