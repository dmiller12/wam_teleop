#pragma once
#include <haptic_wrist/haptic_wrist.h>

#include <boost/asio.hpp>
#include <iostream>
#include <cmath>

#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

template <size_t DOF>
class Leader : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Output<jt_type> wamJPOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Leader(barrett::systems::ExecutionManager* em, haptic_wrist::HapticWrist* hw, const std::string& remoteHost,
                    int rec_port = 5554, int send_port = 5555, const std::string& sysName = "Leader")
        : System(sysName)
        , theirJp(0.0)
        , theirJv(0.0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamJPOutput(this, &jtOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , hw(hw)
        , state(State::INIT) {

        kp.setZero();
        kd.setZero();

        if constexpr (DOF >= 4) {
            kp.template head<4>() << 900, 1000, 400, 200;
            kd.template head<4>() << 10, 6, 3.3, 0.8;
        }

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Leader() {
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
    jp_type wamJP;
    jv_type wamJV;
    Eigen::Matrix<double, DOF + 3, 1> sendJpMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendJvMsg;

    using ReceivedData = typename UDPHandler<DOF + 3>::ReceivedData;

    virtual void operate() {

        // TODO: change back to 1.5, likely need to scale vel as well
        double j5_scale = 1.0;
        double j7_scale = 1.0;

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        haptic_wrist::jp_type wristJP = hw->getPosition();
        haptic_wrist::jp_type wristJV = hw->getVelocity();
        Eigen::Quaterniond hwOrientation  = hw->getOrientation();

        // Eigen::AngleAxisd angle_axis(hwOrientation);

        // std::cout << "Angle: " << angle_axis.angle()
        //           << " rad, Axis: [" << angle_axis.axis().transpose()
        //           << "]" << std::endl;

        sendJpMsg << wamJP, wristJP;
        sendJvMsg << wamJV, wristJV;
        sendJpMsg(4) = j5_scale * sendJpMsg(4);
        sendJpMsg(6) = j7_scale * sendJpMsg(6);

        udp_handler.send(sendJpMsg, sendJvMsg, hwOrientation);

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        auto delay = now - received_data->timestamp;
        if (received_data && (delay <= TIMEOUT_DURATION)) {

            theirJp = received_data->jp.template head<DOF>();
            theirJv = received_data->jv.template head<DOF>();
            theirOrientation = received_data->orientation;
        } else {
            if (state == State::LINKED) {
                auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(delay).count();
                std::cout << "lost link, delay: " << delay_ms << " ms" << std::endl;
                state = State::UNLINKED;
            }
        }

        Eigen::Quaterniond command_orientation;

        switch (state) {
            case State::INIT:
                // Used so haptic wirst holds on moveTo command
                control.setZero();
                command_orientation = hwOrientation.normalized();
                break;
            case State::LINKED:
                command_orientation = theirOrientation.normalized();
                control = compute_control(theirJp, theirJv, wamJP, wamJV);
                break;
            case State::UNLINKED:
                // Changed to unlinked with either timeout or callee.
                control.setZero();
                command_orientation = hwOrientation.normalized();
                break;
        }

        hw->setTarget(command_orientation);
        jtOutputValue->setData(&control);
        printOrientation("Leader", command_orientation);
    }

    jp_type theirJp;
    jp_type theirJv;
    jt_type control;
    Eigen::Quaterniond theirOrientation;
    haptic_wrist::jp_type theirWristJp;

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);
    haptic_wrist::HapticWrist* hw;
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF + 3> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(30);
    State state;
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;

    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jp_type& cur_pos,
                            const jv_type& cur_vel) {
        jt_type pos_term = kp.asDiagonal() * (ref_pos - cur_pos);
        jt_type vel_term = kd.asDiagonal() * (ref_vel - cur_vel);
        return pos_term + vel_term;
    };

    static void printOrientation(const std::string& label, const Eigen::Quaterniond& quat) {
        Eigen::Quaterniond normalized = quat.normalized();
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        Eigen::Matrix3d R = normalized.toRotationMatrix();
        Eigen::Vector3d rpy_rad = R.eulerAngles(0, 1, 2);
        Eigen::Vector3d rpy_deg = rpy_rad * kRadToDeg;
        // std::cout << "[" << label << "] Wrist target RPY (deg): " << rpy_deg.transpose() << std::endl;
    }
};
