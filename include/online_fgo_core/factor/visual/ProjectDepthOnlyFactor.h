#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// #include "../FactorType.h"
#include "include/factor/FactorTypeID.h"

namespace fgo::factor {

class ProjectDepthOnly
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, double> {
  protected:
    gtsam::Point3 measured_i_;
    gtsam::Point3 measured_j_;

    boost::shared_ptr<gtsam::Cal3_S2> K_;
    boost::optional<gtsam::Pose3>
        T_base_cam_;  // assume fixed cam-base extrinsic

    gtsam::Pose3 T_we_;

  public:
    using Base = NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, double>;

    using This = ProjectDepthOnly;
    using shared_ptr = boost::shared_ptr<This>;

  public:
    ProjectDepthOnly(
        const gtsam::Key pose_i,
        const gtsam::Key pose_j,
        const gtsam::Key inv_depth,
        const gtsam::Point3& measured_i,
        const gtsam::Point3& measured_j,
        const gtsam::SharedNoiseModel& model,
        const boost::shared_ptr<gtsam::Cal3_S2>& K,
        const gtsam::Pose3& T_we = gtsam::Pose3(),
        const boost::optional<gtsam::Pose3>& body_P_sensor = boost::none)
        : Base(model, pose_i, pose_j, inv_depth),
          measured_i_{measured_i},
          measured_j_{measured_j},
          K_(K),
          T_base_cam_(body_P_sensor),
          T_we_{T_we} {
    }

    ~ProjectDepthOnly() override = default;

  public:
    gtsam::Vector evaluateError(
        const gtsam::Pose3& p_i,
        const gtsam::Pose3& p_j,
        const double& inv_dep,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none) const override {
        // TODO: @Haoming check this factor
        try {

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

            if (H1) {
                // 2 x 3 x 3 x 6 x 6 x 6 x 6 x 6
                *H1 = HprojectPoint * HPoseFromCam_I * HBase2Cam_I *
                      HIMU2LocalWorld_I;
            }
            if (H2) {
                // 2 x 6 x 6 x 6 x 6 x 6
                *H2 = HprojectPose * HBase2Cam_J * HIMU2LocalWorld_J;
            }
            if (H3) {
                // 2 x 3 x 3 x 3 x 1 x 1 x 3 x 1
                *H3 = HprojectPoint * HPointFromCam_I * HInvDepPointCam_I;
            }

            return error;
        } catch (gtsam::CheiralityException& e) {
            if (H1)
                *H1 = gtsam::Matrix::Zero(2, 6);
            if (H2)
                *H2 = gtsam::Matrix::Zero(2, 6);
            if (H3)
                *H3 = gtsam::Matrix::Zero(2, 1);
        }

        return gtsam::Vector2::Constant(2.0 * K_->fx());
    }

  public:
    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter =
                   gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "ProjectDepthOnly, pt_i = ";
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

};  // class ProjectDepth

}  // namespace fgo::factor

