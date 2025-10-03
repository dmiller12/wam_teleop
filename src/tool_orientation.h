#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/systems/kinematics_base.h>



template<size_t DOF>
class ToolOrientation : public barrett::systems::System, public barrett::systems::KinematicsInput<DOF>,
						public barrett::systems::SingleOutput<Eigen::Quaterniond> {
public:
	ToolOrientation(const std::string& sysName = "ToolOrientation") :
		System(sysName), barrett::systems::KinematicsInput<DOF>(this),
		SingleOutput<Eigen::Quaterniond>(this), rot(), data() {}
	virtual ~ToolOrientation() { mandatoryCleanUp(); }

protected:
	virtual void operate() {
		rot.copyFrom(this->kinInput.getValue().impl->tool->rot_to_world);
		data = rot;

		this->outputValue->setData(&data);
	}

    barrett::math::Matrix<3,3> rot;
	Eigen::Quaterniond data;

private:
	DISALLOW_COPY_AND_ASSIGN(ToolOrientation);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


