/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BetweenFactor.h
 *  @author Frank Dellaert, Viorela Ila
 *  @author Haoming Zhang
 **/
#pragma once

#include <ostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "../FactorTypeID.h"

// This will trigger a LNKxxxx on MSVC, so disable for MSVC build
// Please refer to https://github.com/borglab/gtsam/blob/develop/Using-GTSAM-EXPORT.md
#define BETWEENFACTOR_VISIBILITY GTSAM_EXPORT


namespace fgo::factor {
  using namespace gtsam;

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam gtsam::Pose3 the gtsam::Pose3 type
   * @ingroup slam
   */
  class BetweenPose3Factor : public NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {

    // Check that gtsam::Pose3 type is a testable Lie group
    BOOST_CONCEPT_ASSERT((IsTestable<gtsam::Pose3>));
    BOOST_CONCEPT_ASSERT((IsLieGroup<gtsam::Pose3>));

  public:
    typedef gtsam::Pose3 T;

  private:

    typedef BetweenPose3Factor This;
    typedef NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;

    gtsam::Pose3 measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<BetweenPose3Factor> shared_ptr;

    /// @name Standard Constructors
    /// @{

    /** default constructor - only use for serialization */
    BetweenPose3Factor() {}

    /** Constructor */
    BetweenPose3Factor(Key key1, Key key2, const gtsam::Pose3 &measured,
                       const SharedNoiseModel &model = nullptr) :
      Base(model, key1, key2), measured_(measured) {

    }

    /// @}

    ~BetweenPose3Factor() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// @name Testable
    /// @{

    /// print with optional string
    void print(
      const std::string &s = "",
      const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "BetweenPose3Factor("
                << keyFormatter(this->key1()) << ","
                << keyFormatter(this->key2()) << ")\n";
      traits<T>::Print(measured_, "  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /// assert equality up to a tolerance
    bool equals(const NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<T>::Equals(this->measured_, e->measured_, tol);
    }

    /// @}
    /// @name NoiseModelFactor2 methods
    /// @{

    /// evaluate error, returns vector of errors size of tangent space
    gtsam::Vector evaluateError(const T &p1, const T &p2, boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
      T hx = traits<T>::Between(p1, p2, H1, H2); // h(x)
      // manifold equivalent of h(x)-z -> log(z,h(x))

      typename traits<T>::ChartJacobian::Jacobian Hlocal;
      gtsam::Vector rval = traits<T>::Local(measured_, hx, boost::none, (H1 || H2) ? &Hlocal : 0);
      if (H1) *H1 = Hlocal * (*H1);
      if (H2) *H2 = Hlocal * (*H2);
      return rval;
    }

    /** lifting all related state gtsam::Pose3s in a vector after the ordering for evaluateError **/
    gtsam::Vector liftValuesAsVector(const gtsam::Values &values) {
      const auto poseI = values.at<gtsam::Pose3>(key1());
      const auto poseJ = values.at<gtsam::Pose3>(key2());
      const auto liftedStates = (gtsam::Vector(12) <<
                                                   poseI.rotation().rpy(),
        poseI.translation(),
        poseJ.rotation().rpy(),
        poseJ.translation()).finished();
      return liftedStates;
    }

    gtsam::Values generateValuesFromStateVector(const gtsam::Vector &state) {
      assert(state.size() != 12);
      gtsam::Values values;
      try {
        values.insert(key1(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(0, 0)),
                                           gtsam::Point3(state.block<3, 1>(3, 0))));
        values.insert(key2(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(6, 0)),
                                           gtsam::Point3(state.block<3, 1>(9, 0))));
      }
      catch (std::exception &ex) {
        std::cout << "Factor " << "this factor" << " cannot generate gtsam::Pose3s from state vector " << state
                  << " due to " << ex.what() << std::endl;
      }
      return values;
    }

    /// @}
    /// @name Standard interface
    /// @{

    /// return the measurement
    const gtsam::Pose3 &measured() const {
      return measured_;
    }
    /// @}

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("BetweenPose3Factor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }

    // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    enum {
      NeedsToAlign = (sizeof(gtsam::Pose3) % 16) == 0
    };
  public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  }; // \class BetweenFactor



  /**
   * Binary between constraint - forces between to a given gtsam::Pose3
   * This constraint requires the underlying type to a Lie type
   *
   */
  class BetweenPose3Constraint : public BetweenPose3Factor {
  public:
    typedef boost::shared_ptr<BetweenPose3Constraint> shared_ptr;

    /** Syntactic sugar for constrained version */
    BetweenPose3Constraint(const gtsam::Pose3 &measured, Key key1, Key key2, double mu = 1000.0) :
      BetweenPose3Factor(key1, key2, measured,
                         noiseModel::Constrained::All(traits<gtsam::Pose3>::GetDimension(measured),
                                                      std::abs(mu))) {}

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("BetweenPose3Constraint",
                                          boost::serialization::base_object<BetweenPose3Factor>(*this));
    }
  }; // \class BetweenConstraint

}
namespace gtsam {
  /// traits
  template<>
  struct traits<fgo::factor::BetweenPose3Factor> : public gtsam::Testable<fgo::factor::BetweenPose3Factor> {
  };
  /// traits
  template<>
  struct traits<fgo::factor::BetweenPose3Constraint> : public gtsam::Testable<fgo::factor::BetweenPose3Constraint> {
  };

} /// namespace gtsam

