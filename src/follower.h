#pragma once

#include <boost/asio.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

template <size_t DOF>
class Follower : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<Eigen::Quaterniond> wamOrientationIn;
    Output<jt_type> wamJTOutput;
    Output<Eigen::Quaterniond> wristOrientationOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Follower(barrett::systems::ExecutionManager* em, const std::string& remoteHost, int rec_port = 5554,
                      int send_port = 5555, const std::string& sysName = "Follower")
        : System(sysName)
        , theirJp(0.0)
        , theirJv(0.0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamOrientationIn(this)
        , wamJTOutput(this, &jtOutputValue)
        , wristOrientationOutput(this, &orientationOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , state(State::INIT) {

        kp << 750, 1000, 400, 200, 10, 10, 2.5;
        kd << 8.3, 8, 3.3, 0.8, 0.5, 0.5, 0.05;

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Follower() {
        this->mandatoryCleanUp();
    }

    bool isLinked() const {
        return state == State::LINKED;
    }
    void tryLink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::LINKED;
    }
    void unlink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::UNLINKED;
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<Eigen::Quaterniond>::Value* orientationOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    Eigen::Quaterniond wristOrientation;
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;

    using ReceivedData = typename UDPHandler<DOF>::ReceivedData;

    virtual void operate() {

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wristOrientation = wamOrientationIn.getValue();
        sendJpMsg << wamJP;
        sendJvMsg << wamJV;

        udp_handler.send(sendJpMsg, sendJvMsg, wristOrientation);

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        bool has_remote_orientation = false;
        auto now = std::chrono::steady_clock::now();
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {

            theirJp = received_data->jp;
            theirJv = received_data->jv;
            theirOrientation = received_data->orientation;
            has_remote_orientation = true;
        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        Eigen::Quaterniond* command_orientation_ptr = nullptr;

        switch (state) {
            case State::INIT:
                control.setZero();
                command_orientation_ptr = &wristOrientation;
                break;
            case State::LINKED:
                // Active teleop. Only the callee can transition to LINKED
                control = compute_control(theirJp, theirJv, wamJP, wamJV);
                command_orientation_ptr = &theirOrientation;
                break;
            case State::UNLINKED:
                // Changed to unlinked with either timeout or callee.
                control.setZero();
                command_orientation_ptr = &wristOrientation;
                break;
        }

        jtOutputValue->setData(&control);
        if (command_orientation_ptr != nullptr) {
            orientationOutputValue->setData(command_orientation_ptr);
            Eigen::Quaterniond follower_quat = wristOrientation.normalized();
            if (state == State::LINKED) {
                Eigen::Quaterniond command_quat = command_orientation_ptr->normalized();
                printOrientation("Leader", "target", command_quat);
                printOrientation("Follower", "actual", follower_quat);
                printAlignmentError(command_quat, follower_quat);
                printJointPositions("Leader", theirJp);
                printJointPositions("Follower", wamJP);
            } else if (has_remote_orientation) {
                Eigen::Quaterniond leader_preview = theirOrientation.normalized();
                printOrientation("Leader", "preview", leader_preview);
                printOrientation("Follower", "current", follower_quat);
                printAlignmentError(leader_preview, follower_quat);
                printJointPositions("Leader", theirJp);
                printJointPositions("Follower", wamJP);
            } else {
                printOrientation("Follower", "current", follower_quat);
            }
        }
    }

    jp_type theirJp;
    jp_type theirJv;
    Eigen::Quaterniond theirOrientation;
    jt_type control;

  private:
    DISALLOW_COPY_AND_ASSIGN(Follower);
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);
    State state;
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;

    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jp_type& cur_pos,
                            const jv_type& cur_vel) {
        jt_type pos_term = kp.asDiagonal() * (ref_pos - cur_pos);
        jt_type vel_term = kd.asDiagonal() * (ref_vel - cur_vel);
        return pos_term + vel_term;
    };

    static void printOrientation(const std::string& label, const std::string& measurement_type,
                                 const Eigen::Quaterniond& quat) {
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        Eigen::Matrix3d R = quat.toRotationMatrix();
        Eigen::Vector3d rpy_rad = R.eulerAngles(0, 1, 2);
        Eigen::Vector3d rpy_deg = rpy_rad * kRadToDeg;
        std::cout << "[" << label << "] Wrist " << measurement_type << " RPY (deg): " << rpy_deg.transpose()
                  << std::endl;
    }

    static void printAlignmentError(const Eigen::Quaterniond& target, const Eigen::Quaterniond& actual) {
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        Eigen::Quaterniond delta = target.conjugate() * actual;
        delta.normalize();
        Eigen::AngleAxisd aa(delta);
        double angle_error_deg = aa.angle() * kRadToDeg;
        std::cout << "[Alignment] Angle error (deg): " << angle_error_deg << " Axis: [" << aa.axis().transpose()
                  << "]" << std::endl;
    }

    static void printJointPositions(const std::string& label, const jp_type& joints) {
        Eigen::Matrix<double, 4, 1> first_four = joints.template head<4>();
        std::cout << "[" << label << "] Arm joints 1-4 (rad): " << first_four.transpose() << std::endl;
    }
};
