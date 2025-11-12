#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include "../FactorType.h"
#include "include/factor/FactorTypeID.h"
#include "../../model/gp_interpolator/GPInterpolatorBase.h"

namespace fgo::factor {

class GPInterpolatedProjectDepthOnly
    : public fgo::NoiseModelFactor13<gtsam::Pose3,
                                     gtsam::Vector3,
                                     gtsam::Vector3,
                                     gtsam::Pose3,
                                     gtsam::Vector3,
                                     gtsam::Vector3,
                                     gtsam::Pose3,
                                     gtsam::Vector3,
                                     gtsam::Vector3,
                                     gtsam::Pose3,
                                     gtsam::Vector3,
                                     gtsam::Vector3,
                                     double> {
  protected:
    gtsam::Point3 measured_i_;
    gtsam::Point3 measured_j_;

    boost::shared_ptr<gtsam::Cal3_S2> K_;
    boost::optional<gtsam::Pose3> T_base_cam_;

    gtsam::Pose3 T_we_;

    using GPBase = std::shared_ptr<fgo::models::GPInterpolator>;
    GPBase GPbasePoseI_;
    GPBase GPbasePoseJ_;

  public:
    using Base = NoiseModelFactor13<gtsam::Pose3,
                                    gtsam::Vector3,
                                    gtsam::Vector3,
                                    gtsam::Pose3,
                                    gtsam::Vector3,
                                    gtsam::Vector3,
                                    gtsam::Pose3,
                                    gtsam::Vector3,
                                    gtsam::Vector3,
                                    gtsam::Pose3,
                                    gtsam::Vector3,
                                    gtsam::Vector3,
                                    double>;
    using This = GPInterpolatedProjectDepthOnly;
    using shared_ptr = boost::shared_ptr<This>;

  public:
    GPInterpolatedProjectDepthOnly(
        const gtsam::Key interpolate_pose_i0,
        const gtsam::Key interpolate_vel_i0,
        const gtsam::Key interpolate_omega_i0,
        const gtsam::Key interpolate_pose_i1,
        const gtsam::Key interpolate_vel_i1,
        const gtsam::Key interpolate_omega_i1,
        const gtsam::Key interpolate_pose_j0,
        const gtsam::Key interpolate_vel_j0,
        const gtsam::Key interpolate_omega_j0,
        const gtsam::Key interpolate_pose_j1,
        const gtsam::Key interpolate_vel_j1,
        const gtsam::Key interpolate_omega_j1,
        const gtsam::Key inv_depth,
        const gtsam::Point3& measured_i,
        const gtsam::Point3& measured_j,
        const gtsam::SharedNoiseModel& model,
        const boost::shared_ptr<gtsam::Cal3_S2>& K,
        const std::shared_ptr<fgo::models::GPInterpolator>& interpolatorPoseI,
        const std::shared_ptr<fgo::models::GPInterpolator>& interpolatorPoseJ,
        const gtsam::Pose3& T_we = gtsam::Pose3(),
        const boost::optional<gtsam::Pose3>& body_P_sensor = boost::none)
        : Base(model,
               interpolate_pose_i0,
               interpolate_vel_i0,
               interpolate_omega_i0,
               interpolate_pose_i1,
               interpolate_vel_i1,
               interpolate_omega_i1,
               interpolate_pose_j0,
               interpolate_vel_j0,
               interpolate_omega_j0,
               interpolate_pose_j1,
               interpolate_vel_j1,
               interpolate_omega_j1,
               inv_depth),
          measured_i_{measured_i},
          measured_j_{measured_j},
          K_(K),
          T_base_cam_(body_P_sensor),
          T_we_{T_we},
          GPbasePoseI_(interpolatorPoseI),
          GPbasePoseJ_{interpolatorPoseJ} {
    }

    ~GPInterpolatedProjectDepthOnly() override = default;

  public:
    gtsam::Vector evaluateError(
        const gtsam::Pose3& interpolate_pose_i0,
        const gtsam::Vector3& interpolate_vel_i0,
        const gtsam::Vector3& interpolate_omega_i0,
        const gtsam::Pose3& interpolate_pose_i1,
        const gtsam::Vector3& interpolate_vel_i1,
        const gtsam::Vector3& interpolate_omega_i1,
        const gtsam::Pose3& interpolate_pose_j0,
        const gtsam::Vector3& interpolate_vel_j0,
        const gtsam::Vector3& interpolate_omega_j0,
        const gtsam::Pose3& interpolate_pose_j1,
        const gtsam::Vector3& interpolate_vel_j1,
        const gtsam::Vector3& interpolate_omega_j1,
        const double& inv_dep,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none,
        boost::optional<gtsam::Matrix&> H4 = boost::none,
        boost::optional<gtsam::Matrix&> H5 = boost::none,
        boost::optional<gtsam::Matrix&> H6 = boost::none,
        boost::optional<gtsam::Matrix&> H7 = boost::none,
        boost::optional<gtsam::Matrix&> H8 = boost::none,
        boost::optional<gtsam::Matrix&> H9 = boost::none,
        boost::optional<gtsam::Matrix&> H10 = boost::none,
        boost::optional<gtsam::Matrix&> H11 = boost::none,
        boost::optional<gtsam::Matrix&> H12 = boost::none,
        boost::optional<gtsam::Matrix&> H13 = boost::none) const override {
        // TODO: @Haoming check this factor

        try {
            // calc p_i
            gtsam::Pose3 p_i;
            gtsam::Matrix HINTERPOLATE1i, HINTERPOLATE2i, HINTERPOLATE3i,
                HINTERPOLATE4i, HINTERPOLATE5i, HINTERPOLATE6i;
            if (H1 || H2 || H3 || H4 || H5 || H6) {
                p_i = GPbasePoseI_->interpolatePose(interpolate_pose_i0,
                                                    interpolate_vel_i0,
                                                    interpolate_omega_i0,
                                                    interpolate_pose_i1,
                                                    interpolate_vel_i1,
                                                    interpolate_omega_i1,
                                                    HINTERPOLATE1i,
                                                    HINTERPOLATE2i,
                                                    HINTERPOLATE3i,
                                                    HINTERPOLATE4i,
                                                    HINTERPOLATE5i,
                                                    HINTERPOLATE6i);
            } else {
                p_i = GPbasePoseI_->interpolatePose(interpolate_pose_i0,
                                                    interpolate_vel_i0,
                                                    interpolate_omega_i0,
                                                    interpolate_pose_i1,
                                                    interpolate_vel_i1,
                                                    interpolate_omega_i1);
            }

            // calc p_j
            gtsam::Pose3 p_j;
            gtsam::Matrix HINTERPOLATE7j, HINTERPOLATE8j, HINTERPOLATE9j,
                HINTERPOLATE10j, HINTERPOLATE11j, HINTERPOLATE12j;
            if (H7 || H8 || H9 || H10 || H11 || H12) {
                p_j = GPbasePoseJ_->interpolatePose(interpolate_pose_j0,
                                                    interpolate_vel_j0,
                                                    interpolate_omega_j0,
                                                    interpolate_pose_j1,
                                                    interpolate_vel_j1,
                                                    interpolate_omega_j1,
                                                    HINTERPOLATE7j,
                                                    HINTERPOLATE8j,
                                                    HINTERPOLATE9j,
                                                    HINTERPOLATE10j,
                                                    HINTERPOLATE11j,
                                                    HINTERPOLATE12j);
            } else {
                p_j = GPbasePoseJ_->interpolatePose(interpolate_pose_j0,
                                                    interpolate_vel_j0,
                                                    interpolate_omega_j0,
                                                    interpolate_pose_j1,
                                                    interpolate_vel_j1,
                                                    interpolate_omega_j1);
            }

            // using p_i & p_j
            // same as ProjectDepthOnly
            // except jacobians
            gtsam::Matrix HIMU2LocalWorld_I = gtsam::Matrix66::Identity();
            const auto T_w_base_i =
                T_we_.transformPoseFrom(p_i, boost::none, HIMU2LocalWorld_I);

            gtsam::Matrix HIMU2LocalWorld_J = gtsam::Matrix66::Identity();
            const auto T_w_base_j =
                T_we_.transformPoseFrom(p_j, boost::none, HIMU2LocalWorld_J);

            gtsam::Matrix HBase2Cam_I = gtsam::Matrix66::Identity();
            gtsam::Matrix HBase2Cam_J = gtsam::Matrix66::Identity();
            gtsam::Pose3 T_w_cam_i, T_w_cam_j;
            if (T_base_cam_.has_value()) {
                T_w_cam_i =
                    T_w_base_i.transformPoseFrom(*T_base_cam_, HBase2Cam_I);
                T_w_cam_j =
                    T_w_base_j.transformPoseFrom(*T_base_cam_, HBase2Cam_J);
            } else {
                T_w_cam_i = T_w_base_i;
                T_w_cam_j = T_w_base_j;
            }

            gtsam::Point3 pt_in_cam_i = measured_i_ / inv_dep;
            gtsam::Matrix31 HInvDepPointCam_I =
                -1.0 / (inv_dep * inv_dep) * measured_i_;

            gtsam::Matrix HPoseFromCam_I;
            gtsam::Matrix HPointFromCam_I;
            gtsam::Point3 pt_in_w = T_w_cam_i.transformFrom(pt_in_cam_i,
                                                            HPoseFromCam_I,
                                                            HPointFromCam_I);

            // gtsam::Matrix HPoseToCam_J;
            // gtsam::Matrix HPointToCam_J;
            // gtsam::Point3 pt_in_cam_j =
            //     T_w_cam_j.transformTo(pt_in_w, HPoseToCam_J, HPointToCam_j);

            gtsam::PinholeCamera<gtsam::Cal3_S2> camera_j(T_w_cam_j, *K_);
            gtsam::Matrix HprojectPose;
            gtsam::Matrix HprojectPoint;
            gtsam::Point2 error(camera_j.project(pt_in_w,
                                                 HprojectPose,
                                                 HprojectPoint,
                                                 boost::none) -
                                measured_j_.head(2));

            // update jacobian for interpolation pose I
            if (H1) {
                *H1 = HprojectPoint * HPoseFromCam_I * HBase2Cam_I *
                      HIMU2LocalWorld_I * HINTERPOLATE1i;
            }
            if (H2) {
                *H2 = HprojectPoint * HPoseFromCam_I * HBase2Cam_I *
                      HIMU2LocalWorld_I * HINTERPOLATE2i;
            }
            if (H3) {
                *H3 = HprojectPoint * HPoseFromCam_I * HBase2Cam_I *
                      HIMU2LocalWorld_I * HINTERPOLATE3i;
            }
            if (H4) {
                *H4 = HprojectPoint * HPoseFromCam_I * HBase2Cam_I *
                      HIMU2LocalWorld_I * HINTERPOLATE4i;
            }
            if (H5) {
                *H5 = HprojectPoint * HPoseFromCam_I * HBase2Cam_I *
                      HIMU2LocalWorld_I * HINTERPOLATE5i;
            }
            if (H6) {
                *H6 = HprojectPoint * HPoseFromCam_I * HBase2Cam_I *
                      HIMU2LocalWorld_I * HINTERPOLATE6i;
            }

            // update jacobian for interpolation pose J
            if (H7) {
                *H7 = HprojectPose * HBase2Cam_J * HIMU2LocalWorld_J *
                      HINTERPOLATE7j;
            }
            if (H8) {
                *H8 = HprojectPose * HBase2Cam_J * HIMU2LocalWorld_J *
                      HINTERPOLATE8j;
            }
            if (H9) {
                *H9 = HprojectPose * HBase2Cam_J * HIMU2LocalWorld_J *
                      HINTERPOLATE9j;
            }
            if (H10) {
                *H10 = HprojectPose * HBase2Cam_J * HIMU2LocalWorld_J *
                       HINTERPOLATE10j;
            }
            if (H11) {
                *H11 = HprojectPose * HBase2Cam_J * HIMU2LocalWorld_J *
                       HINTERPOLATE11j;
            }
            if (H12) {
                *H12 = HprojectPose * HBase2Cam_J * HIMU2LocalWorld_J *
                       HINTERPOLATE12j;
            }

            // update jacobian for inv_dep
            if (H13) {
                *H13 = HprojectPoint * HPointFromCam_I * HInvDepPointCam_I;
            }

        } catch (gtsam::CheiralityException& e) {
            // interpolate i
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

            // interpolate j
            if (H7)
                *H1 = gtsam::Matrix::Zero(2, 3);
            if (H8)
                *H2 = gtsam::Matrix::Zero(2, 3);
            if (H9)
                *H3 = gtsam::Matrix::Zero(2, 3);
            if (H10)
                *H4 = gtsam::Matrix::Zero(2, 3);
            if (H11)
                *H5 = gtsam::Matrix::Zero(2, 3);
            if (H12)
                *H6 = gtsam::Matrix::Zero(2, 3);

            // inv_dep
            if (H13)
                *H6 = gtsam::Matrix::Zero(2, 1);
        }
        return gtsam::Vector2::Constant(2.0 * K_->fx());
    }

  public:
    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter =
                   gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "GPInterpolatedProjectDepthOnly, pt_i = ";
        gtsam::traits<gtsam::Point3>::Print(measured_i_);
        std::cout << s << " pt_j = ";
        gtsam::traits<gtsam::Point3>::Print(measured_j_);
        if (this->T_base_cam_)
            this->T_base_cam_->print("  sensor pose in body frame: ");
        Base::print("", keyFormatter);
    }

    bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
        const This* e = dynamic_cast<const This*>(&p);
        return e && Base::equals(p, tol) &&
               gtsam::traits<gtsam::Point3>::Equals(this->measured_i_,
                                                    e->measured_i_,
                                                    tol) &&
               gtsam::traits<gtsam::Point3>::Equals(this->measured_j_,
                                                    e->measured_j_,
                                                    tol) &&
               this->K_->equals(*e->K_, tol) &&
               ((!T_base_cam_ && !e->T_base_cam_) ||
                (T_base_cam_ && e->T_base_cam_ &&
                 T_base_cam_->equals(*e->T_base_cam_))) &&
               gtsam::traits<gtsam::Pose3>::Equals(this->T_we_,
                                                   this->T_we_,
                                                   tol);
    }

  public:
    const gtsam::Point3& measuredI() const {
        return measured_i_;
    }

    const gtsam::Point3& measuredJ() const {
        return measured_j_;
    }

    const boost::shared_ptr<gtsam::Cal3_S2> calibration() const {
        return K_;
    }

    const boost::optional<gtsam::Pose3>& body_P_sensor() const {
        return T_base_cam_;
    }

    // inline bool verboseCheirality() const {
    //     return verboseCheirality_;
    // }
    //
    // inline bool throwCheirality() const {
    //     return throwCheirality_;
    // }

  public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};  // class GPInterpolatedProjectDepthOnly

}  // namespace fgo::factor

