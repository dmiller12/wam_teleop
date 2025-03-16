#pragma once

#include "haptic_wrist/haptic_wrist.h"
#include <barrett/units.h>

#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems.h>

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

class ToolFrameCb : public barrett::systems::System, public barrett::systems::SingleInput<pose_type> {
  public:
    explicit ToolFrameCb(haptic_wrist::HapticWrist *hw, const std::string &sysName = "ToolFrameCb")
        : System(sysName), SingleInput<pose_type>(this), hw(hw) {}
    virtual ~ToolFrameCb() { mandatoryCleanUp(); }

  protected:
    haptic_wrist::HapticWrist *hw;

    virtual void operate() {

        auto wamPose = this->input.getValue();
        hw->setWristToBase(posQuatToTransform(boost::get<0>(wamPose), boost::get<1>(wamPose)));
    }
    Eigen::Matrix4d posQuatToTransform(const Eigen::Vector3d &position, const Eigen::Quaterniond &quaternion) {
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

        Eigen::Matrix3d R = quaternion.toRotationMatrix().transpose();
        transformation.block<3, 3>(0, 0) = R;

        // Set translation
        transformation.block<3, 1>(0, 3) = -R * position;

        return transformation;
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(ToolFrameCb);
};

