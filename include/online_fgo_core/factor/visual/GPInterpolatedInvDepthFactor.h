#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/slam/InvDepthFactor3.h>
#include <gtsam_unstable/slam/InvDepthFactorVariant1.h>
#include <gtsam_unstable/slam/InvDepthFactorVariant2.h>
#include <gtsam_unstable/slam/InvDepthFactorVariant3.h>

#include "../FactorType.h"
#include "../FactorTypeID.h"
#include "../../model/gp_interpolator/GPInterpolatorBase.h"

namespace fgo::factor {

class GPInterpolatedInvDepthFactor
    : public fgo::NoiseModelFactor7<gtsam::Pose3,
                                    gtsam::Vector3,
                                    gtsam::Vector3,
                                    gtsam::Pose3,
                                    gtsam::Vector3,
                                    gtsam::Vector3,
                                    gtsam::Vector6> {
  protected:
    gtsam::Point2 measured_;
    gtsam::Pose3 T_we;  ;
    boost::shared_ptr<gtsam::Cal3_S2> K_;
    boost::optional<gtsam::Pose3> T_base_cam;

    using GPBase = std::shared_ptr<fgo::models::GPInterpolator>;
    GPBase GPbasePose_;

  public:
    using Base = NoiseModelFactor7<gtsam::Pose3,
                                   gtsam::Vector3,
                                   gtsam::Vector3,
                                   gtsam::Pose3,
                                   gtsam::Vector3,
                                   gtsam::Vector3,
                                   gtsam::Vector6>;
    using This = GPInterpolatedInvDepthFactor;
    using shared_ptr = boost::shared_ptr<This>;

  public:
    GPInterpolatedInvDepthFactor(
        const gtsam::Key pose1i,
        const gtsam::Key vel1i,
        const gtsam::Key omega1i,
        const gtsam::Key pose1j,
        const gtsam::Key vel1j,
        const gtsam::Key omega1j,
        const gtsam::Key landmark,
        const gtsam::Point2& measured,
        const gtsam::SharedNoiseModel& model,
        const boost::shared_ptr<gtsam::Cal3_S2>& K,
        const std::shared_ptr<fgo::models::GPInterpolator>& interpolatorPose,
        const gtsam::Pose3& T_anc = gtsam::Pose3(),
        const boost::optional<gtsam::Pose3> &body_P_sensor = boost::none)
        : Base(model, pose1i, vel1i, omega1i, pose1j, vel1j, omega1j, landmark),
          measured_(measured),
          T_we(T_anc),
          K_(K),
          T_base_cam(body_P_sensor),
          GPbasePose_(interpolatorPose) {
    }

  public:
    gtsam::Vector inverseDepthError(const gtsam::Pose3& pose,
                                    const gtsam::Vector6& landmark) const {
        try {
            // Calculate the 3D coordinates of the landmark in the world frame
            const double x = landmark(0), y = landmark(1), z = landmark(2);
            const double theta = landmark(3), phi = landmark(4), rho = landmark(5);
            const gtsam::Point3 world_P_landmark =
                gtsam::Point3(x, y, z) +
                gtsam::Point3(cos(theta) * cos(phi) / rho,
                              sin(theta) * cos(phi) / rho,
                              sin(phi) / rho);
            // Project landmark into Pose2
            const gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pose, *K_);
            return camera.project(world_P_landmark) - measured_;
        } catch (gtsam::CheiralityException& e) {
            std::cout << e.what() << ": Inverse Depth Landmark ["
                      << gtsam::DefaultKeyFormatter(this->key2()) << "]"
                      << " moved behind camera ["
                      << gtsam::DefaultKeyFormatter(this->key1()) << "]"
                      << std::endl;
            return gtsam::Vector::Ones(2) * 2.0 * K_->fx();
        }
        return (gtsam::Vector(1) << 0.0).finished();
    }

    /// Evaluate error h(x)-z and optionally derivatives
    gtsam::Vector evaluateError(
        const gtsam::Pose3& p1i,
        const gtsam::Vector3& v1i,
        const gtsam::Vector3& omega1i,
        const gtsam::Pose3& p1j,
        const gtsam::Vector3& v1j,
        const gtsam::Vector3& omega1j,
        const gtsam::Vector6& landmark,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none,
        boost::optional<gtsam::Matrix&> H4 = boost::none,
        boost::optional<gtsam::Matrix&> H5 = boost::none,
        boost::optional<gtsam::Matrix&> H6 = boost::none,
        boost::optional<gtsam::Matrix&> H7 = boost::none) const override {
        gtsam::Pose3 T_e_base;
        gtsam::Matrix Hint11_P, Hint12_P, Hint13_P, Hint14_P, Hint15_P,
            Hint16_P, Hpose1, Hpose2;
        if (H1 || H2 || H3 || H4 || H5 || H6) {
            T_e_base = GPbasePose_->interpolatePose(p1i,
                                                     v1i,
                                                     omega1i,
                                                     p1j,
                                                     v1j,
                                                     omega1j,
                                                     Hint11_P,
                                                     Hint12_P,
                                                     Hint13_P,
                                                     Hint14_P,
                                                     Hint15_P,
                                                     Hint16_P);
        } else {
            T_e_base = GPbasePose_->interpolatePose(p1i,
                                                     v1i,
                                                     omega1i,
                                                     p1j,
                                                     v1j,
                                                     omega1j);
        }

        gtsam::Matrix HIMU2LocalWorld, HIMU2Sensor = gtsam::Matrix66::Identity();
        const auto T_w_base = T_we.transformPoseFrom(T_e_base, boost::none, HIMU2LocalWorld);
        gtsam::Pose3 T_w_cam;
        if(T_base_cam)
            // TODO: @haoming change HIMU2LocalWorld to HIMU2Sensor below, double check here
            T_w_cam = T_w_base.transformPoseFrom(*T_base_cam, HIMU2Sensor);
        else
            T_w_cam = T_w_base;

        if (H1 || H2 || H3 || H4 || H5 || H6) {
            gtsam::Matrix poseH =
                gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                    std::bind(&GPInterpolatedInvDepthFactor::inverseDepthError,
                              this,
                              std::placeholders::_1,
                              landmark),
                    T_w_cam);
            if (H1) {
                *H1 = poseH * HIMU2Sensor * HIMU2LocalWorld * Hint11_P;
            }
            if (H2) {
                *H2 = poseH * HIMU2Sensor * HIMU2LocalWorld * Hint12_P;
            }
            if (H3) {
                *H3 = poseH * HIMU2Sensor * HIMU2LocalWorld * Hint13_P;
            }
            if (H4) {
                *H4 = poseH * HIMU2Sensor * HIMU2LocalWorld * Hint14_P;
            }
            if (H5) {
                *H5 = poseH * HIMU2Sensor * HIMU2LocalWorld * Hint15_P;
            }
            if (H6) {
                *H6 = poseH * HIMU2Sensor * HIMU2LocalWorld * Hint16_P;
            }
        }

        if (H7) {
            // landmark
            (*H7) = numericalDerivative11<gtsam::Vector, gtsam::Vector6>(
                std::bind(&GPInterpolatedInvDepthFactor::inverseDepthError,
                          this,
                          T_w_cam,
                          std::placeholders::_1),
                landmark);
        }

        return inverseDepthError(T_w_cam, landmark);
    }

  public:
    void print(const std::string& s = "GPInterpolatedInvDepthFactor",
               const gtsam::KeyFormatter& keyFormatter =
                   gtsam::DefaultKeyFormatter) const override {
        Base::print(s, keyFormatter);
        gtsam::traits<gtsam::Point2>::Print(measured_, s + ".z");
    }

    bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
        const This* e = dynamic_cast<const This*>(&p);
        return e && Base::equals(p, tol) &&
               gtsam::traits<gtsam::Point2>::Equals(this->measured_,
                                                    e->measured_,
                                                    tol) &&
               this->K_->equals(*e->K_, tol);
    }

  public:
    const gtsam::Point2& measured() const {
        return measured_;
    }

    const boost::shared_ptr<gtsam::Cal3_S2> calibration() const {
        return K_;
    }
};  // class GPInterpolatedInvDepthFactor

}  // namespace fgo::factor

