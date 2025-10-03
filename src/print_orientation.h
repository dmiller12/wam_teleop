#include <Eigen/Dense>
#include <iostream>
#include <string>

#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
//

class PrintOrientation : public barrett::systems::System, public barrett::systems::SingleInput<Eigen::Quaterniond> {
public:
    explicit PrintOrientation(barrett::systems::ExecutionManager* em,
                              const std::string& prependedLabel = "",
                              std::ostream& ostream = std::cout, const std::string& sysName = "PrintQuaternion") :
		System(sysName), SingleInput<Eigen::Quaterniond>(this), label(prependedLabel), os(ostream)
	{
		if (em != NULL) {
			em->startManaging(*this);
		}
	}
	virtual ~PrintOrientation() {
		mandatoryCleanUp();
	}
protected:
	std::string label;
	std::ostream& os;
    virtual void operate() {
        const Eigen::Quaterniond& q = this->input.getValue();

        Eigen::AngleAxisd angle_axis(q);

        // 3. Print the formatted output
        // The .transpose() method prints the axis vector horizontally
        os << label << "Angle: " << angle_axis.angle()
                  << " rad, Axis: [" << angle_axis.axis().transpose()
                  << "]" << std::endl;
    }
private:
	DISALLOW_COPY_AND_ASSIGN(PrintOrientation);
};

