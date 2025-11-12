#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include "../FactorType.h"
#include "include/factor/FactorTypeID.h"
#include "../../model/gp_interpolator/GPInterpolatorBase.h"

namespace fgo::factor {

class GPInterpolatedProjectFactor
    : public fgo::NoiseModelFactor7<gtsam::Pose3,
                                    gtsam::Vector3,
                                    gtsam::Vector3,
                                    gtsam::Pose3,
                                    gtsam::Vector3,
                                    gtsam::Vector3,
                                    gtsam::Point3> {
  protected:
    gtsam::Point2 measured_;
    gtsam::Pose3 T_we_;
    boost::shared_ptr<gtsam::Cal3_S2> K_;
    boost::optional<gtsam::Pose3> T_base_cam_;

    using GPBase = std::shared_ptr<fgo::models::GPInterpolator>;
    GPBase GPbasePose_;

    bool throwCheirality_;
    bool verboseCheirality_;

  public:
    using Base = NoiseModelFactor7<gtsam::Pose3,
                                   gtsam::Vector3,
                                   gtsam::Vector3,
                                   gtsam::Pose3,
                                   gtsam::Vector3,
                                   gtsam::Vector3,
                                   gtsam::Point3>;
    using This = GPInterpolatedProjectFactor;
    using shared_ptr = boost::shared_ptr<This>;

  public:
    GPInterpolatedProjectFactor(
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
        const boost::optional<gtsam::Pose3>& body_P_sensor = boost::none)
        : Base(model, pose1i, vel1i, omega1i, pose1j, vel1j, omega1j, landmark),
          measured_(measured),
          K_(K),
          T_base_cam_(body_P_sensor),
          T_we_(T_anc),
          GPbasePose_(interpolatorPose),
          throwCheirality_(false),
          verboseCheirality_(false) {
    }

    GPInterpolatedProjectFactor(
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
        const bool throwCheirality,
        const bool verboseCheirality,
        const gtsam::Pose3& T_we = gtsam::Pose3(),
        const boost::optional<gtsam::Pose3>& body_P_sensor = boost::none)
        : Base(model, pose1i, vel1i, omega1i, pose1j, vel1j, omega1j, landmark),
          measured_(measured),
          K_(K),
          T_base_cam_(body_P_sensor),
          T_we_(T_we),
          GPbasePose_(interpolatorPose),
          throwCheirality_(throwCheirality),
          verboseCheirality_(verboseCheirality) {
    }

    ~GPInterpolatedProjectFactor() override = default;

  public:
    gtsam::Vector evaluateError(
        const gtsam::Pose3& p1i,
        const gtsam::Vector3& v1i,
        const gtsam::Vector3& omega1i,
        const gtsam::Pose3& p1j,
        const gtsam::Vector3& v1j,
        const gtsam::Vector3& omega1j,
        const gtsam::Point3& landmark,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none,
        boost::optional<gtsam::Matrix&> H4 = boost::none,
        boost::optional<gtsam::Matrix&> H5 = boost::none,
        boost::optional<gtsam::Matrix&> H6 = boost::none,
        boost::optional<gtsam::Matrix&> H7 = boost::none) const override {
        try {
            // interpolation
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

            gtsam::Matrix HIMU2LocalWorld,
                HIMU2Sensor = gtsam::Matrix66::Identity();
            const auto T_w_base =
                T_we_.transformPoseFrom(T_e_base, boost::none, HIMU2LocalWorld);
            gtsam::Pose3 T_w_cam;
            if (T_base_cam_)
                T_w_cam = T_w_base.transformPoseFrom(*T_base_cam_, HIMU2Sensor);
            else
                T_w_cam = T_w_base;

            gtsam::Matrix projectPoseH;
            gtsam::PinholeCamera<gtsam::Cal3_S2> camera(T_w_cam, *K_);

            gtsam::Point2 reprojectionError(
                camera.project(landmark, projectPoseH, H7, boost::none) -
                measured_);

            if (H1) {
                *H1 = projectPoseH * HIMU2Sensor * HIMU2LocalWorld *
                      Hint11_P;  // 2 x 6 x 6 x 6 x 6 x 6
            }
            if (H2) {
                *H2 = projectPoseH * HIMU2Sensor * HIMU2LocalWorld *
                      Hint12_P;  // 2 x 6 x 6 x 6 x 6 x 3
            }
            if (H3) {
                *H3 = projectPoseH * HIMU2Sensor * HIMU2LocalWorld *
                      Hint13_P;  // 2 x 6 x 6 x 6 x 6 x 3
            }
            if (H4) {
                *H4 = projectPoseH * HIMU2Sensor * HIMU2LocalWorld *
                      Hint14_P;  // 2 x 6 x 6 x 6 x 6 x 6
            }
            if (H5) {
                *H5 = projectPoseH * HIMU2Sensor * HIMU2LocalWorld *
                      Hint15_P;  // 2 x 6 x 6 x 6 x 6 x 3
            }
            if (H6) {
                *H6 = projectPoseH * HIMU2Sensor * HIMU2LocalWorld *
                      Hint16_P;  // 2 x 6 x 6 x 6 x 6 x 3
            }
            return reprojectionError;

        } catch (gtsam::CheiralityException& e) {
            if (H1)
                *H1 = gtsam::Matrix::Zero(2, 3);
            if (H2)
                *H2 = gtsam::Matrix::Zero(2, 3);
            if (H3)
                *H3 = gtsam::Matrix::Zero(2, 3);
            if (H4)
                *H4 = gtsam::Matrix::Zero(2, 3);
            if (H5)
                *H5 = gtsam::Matrix::Zero(2, 3);
            if (H6)
                *H6 = gtsam::Matrix::Zero(2, 3);
            if (H7)
                *H7 = gtsam::Matrix::Zero(2, 3);
            if (verboseCheirality_)
                std::cout << e.what() << ": Landmark "
                          << gtsam::DefaultKeyFormatter(this->key7())
                          << " moved behind camera";
            if (throwCheirality_)
                throw gtsam::CheiralityException(this->key7());
        }
        return gtsam::Vector2::Constant(2.0 * K_->fx());
    }

  public:
    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter =
                   gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "GPInterpolatedProjectFactor, z = ";
        gtsam::traits<gtsam::Point2>::Print(measured_);
        if (this->T_base_cam_)
            this->T_base_cam_->print("  sensor pose in body frame: ");
        Base::print("", keyFormatter);
    }

    bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
        const This* e = dynamic_cast<const This*>(&p);
        return e && Base::equals(p, tol) &&
               gtsam::traits<gtsam::Point2>::Equals(this->measured_,
                                                    e->measured_,
                                                    tol) &&
               this->K_->equals(*e->K_, tol) &&
               ((!T_base_cam_ && !e->T_base_cam_) ||
                (T_base_cam_ && e->T_base_cam_ &&
                 T_base_cam_->equals(*e->T_base_cam_)));
    }

  public:
    const gtsam::Point2& measured() const {
        return measured_;
    }

    const boost::shared_ptr<gtsam::Cal3_S2> calibration() const {
        return K_;
    }

    const boost::optional<gtsam::Pose3>& body_P_sensor() const {
        return T_base_cam_;
    }

    inline bool verboseCheirality() const {
        return verboseCheirality_;
    }

    inline bool throwCheirality() const {
        return throwCheirality_;
    }

  public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)

};  // class GPInterpolatedProjectFactor

}  // namespace fgo::factor

