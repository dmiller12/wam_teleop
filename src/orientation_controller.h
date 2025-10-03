
/*
        Copygight 2009, 2010 Barrett Technology <support@barrett.com>

        This file is part of libbarrett.

        This version of libbarrett is free software: you can redistribute it
        and/or modify it under the terms of the GNU General Public License as
        published by the Free Software Foundation, either version 3 of the
        License, or (at your option) any later version.

        This version of libbarrett is distributed in the hope that it will be
        useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License along
        with this version of libbarrett.  If not, see
        <http://www.gnu.org/licenses/>.

        Further, non-binding information about licensing is available at:
        <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * tool_orientation_controller.h
 *
 *  Created on: Jan 22, 2010
 *      Author: dc
 */

#pragma once

#include <Eigen/Geometry>
#include <gsl/gsl_blas.h>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/detail/libconfig_utils.h>
#include <barrett/math/utils.h>
#include <barrett/systems/abstract/controller.h>
#include <barrett/systems/kinematics_base.h>
#include <barrett/units.h>

template <size_t DOF>
class WristOrientationController
    : public barrett::systems::Controller<Eigen::Quaterniond, typename barrett::units::JointTorques<DOF>::type>,
      public barrett::systems::KinematicsInput<DOF> {

    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    typedef typename ::barrett::units::JointTorques<3>::type jt_wrist_type;

  public:
    explicit WristOrientationController(const std::string& sysName = "ToolOrientationController")
        : barrett::systems::Controller<Eigen::Quaterniond, jt_type>(sysName)
        , barrett::systems::KinematicsInput<DOF>(this)
        , kp(0.0)
        , kd(0.0) {
    }
    explicit WristOrientationController(const libconfig::Setting& setting,
                                        const std::string& sysName = "ToolOrientationController")
        : barrett::systems::Controller<Eigen::Quaterniond, jt_type>(sysName)
        , barrett::systems::KinematicsInput<DOF>(this)
        , kp(0.0)
        , kd(0.0) {
        setFromConfig(setting);
    }
    virtual ~WristOrientationController() {
        this->mandatoryCleanUp();
    }

    void setFromConfig(const libconfig::Setting& setting) {
        setKp(barrett::detail::numericToDouble(setting["kp"]));
        setKd(barrett::detail::numericToDouble(setting["kd"]));
    }
    void setKp(double proportionalGain) {
        kp = proportionalGain;
    }
    void setKd(double derivitiveGain) {
        kd = derivitiveGain;
    }

    double getKp() const {
        return kp;
    }
    double getKd() const {
        return kd;
    }

  protected:
    double kp;
    double kd;

    Eigen::AngleAxisd error;
    ct_type ct;

    jt_type out;

    virtual void operate() {
        error = this->referenceInput.getValue() * this->feedbackInput.getValue().inverse();  // I think
        double angle = error.angle();
        // TODO(dc): I looked into Eigen's implementation and noticed that angle will always be between 0 and 2*pi. We
        // should test for this so if Eigen changes, we notice.
        if (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }

        if (barrett::math::abs(angle) > 3.13) { // a little dead-zone near the discontinuity at +/-180 degrees
            ct.setZero();
        } else {
            ct = error.axis() * angle * kp;
        }
        gsl_blas_daxpy(-kd, this->kinInput.getValue().impl->tool_velocity_angular, ct.asGslType());

        gsl_blas_dgemv(CblasTrans, 1.0,
                        this->kinInput.getValue().impl->tool_jacobian_angular,
                        ct.asGslType(), 0.0, out.asGslType());

        this->controlOutputValue->setData(&out);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(WristOrientationController);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
