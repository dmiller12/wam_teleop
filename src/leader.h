#pragma once
#include <haptic_wrist/haptic_wrist.h>
#include <boost/optional.hpp>
#include "gripper/magnum_opus/magnum_gripper.h"

#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <cstdint>

#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

using namespace gripper::magnum_opus;

template <size_t DOF = 3>
class Leader : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Output<jp_type> wamJPOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Leader(barrett::systems::ExecutionManager* em, haptic_wrist::HapticWrist* hw, MagnumGripper* gripper,
                    const std::string& remoteHost, int rec_port = 5554, int send_port = 5555,
                    const std::string& sysName = "Leader")
        : System(sysName)
        , theirJp(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamJPOutput(this, &jpOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , hw(hw)
        , gripper(gripper)
        , state(State::INIT) {

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
    typename Output<jp_type>::Value* jpOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    Eigen::Matrix<double, DOF + 3, 1> sendJpMsg;

    float joy_x = 0.0f;
    float trigger = 0;
    bool bumper_pressed = 0;
    const double trigger_rest_pos = 0.25;
    float target_velocity = 0.3;
    const float torque_scaling = 1.5;
    const float minStiffness = 0.15;
    const float maxStiffness = 1.0;

    const float alpha = 0.15f;
    float smoothed_torque = 0.0f;

    using ReceivedData = typename UDPHandler<DOF + 3>::ReceivedData;

    virtual void operate() {

        // TODO: change back to 1.5 when recalibrated for this setup
        const double j5_scale = 1.0;

        if (boost::optional<haptic_wrist::handle_type> opt_handle = hw->getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle;
            joy_x = static_cast<float>(handle[0]);
            trigger = static_cast<float>(handle[3]);
            bumper_pressed = static_cast<int>(handle[2]) == 1;

            if (trigger > trigger_rest_pos) {
                gripper->setVelocity(target_velocity * trigger);
            } else if (bumper_pressed) {
                gripper->setVelocity(-target_velocity);
            } else {
                gripper->setVelocity(0.0f);
            }
        }

        gripper->controlLoopCallback();
        GripperState gripper_state = gripper->getLatestState();

        smoothed_torque = (alpha * gripper_state.torque) + ((1.0f - alpha) * smoothed_torque);
        if (smoothed_torque > minStiffness) {
            float dynamicStiffness = smoothed_torque * torque_scaling * (maxStiffness - minStiffness) + minStiffness;
            float raw_haptics = 255.0f * dynamicStiffness;
            if (raw_haptics > 255.0f) {
                raw_haptics = 255.0f;
            }
            hw->setTriggerHaptics(static_cast<uint8_t>(raw_haptics));
        } else {
            hw->setTriggerHaptics(0);
        }

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        haptic_wrist::jp_type wristJP = hw->getPosition();

        sendJpMsg.template head<DOF>() = wamJP;
        sendJpMsg(DOF + 0) = wristJP[0];
        sendJpMsg(DOF + 1) = wristJP[1];
        // J7 channel carries joystick command for follower-side hybrid control.
        sendJpMsg(DOF + 2) = joy_x;

        sendJpMsg(DOF + 0) *= j5_scale;

        udp_handler.send(sendJpMsg);

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {
            theirJp = received_data->jp.template head<DOF>();

            theirWristJp = hw->getPosition();
            if (theirWristJp.size() > 0) {
                theirWristJp[0] = received_data->jp(DOF + 0) / j5_scale;
            }
            if (theirWristJp.size() > 1) {
                theirWristJp[1] = received_data->jp(DOF + 1);
            }
        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        switch (state) {
            case State::INIT:
                jpOutputValue->setData(&wamJP);
                break;
            case State::LINKED:
                hw->setTarget(theirWristJp);
                jpOutputValue->setData(&theirJp);
                break;
            case State::UNLINKED:
                hw->setTarget(wristJP);
                jpOutputValue->setData(&wamJP);
                break;
        }
    }

    jp_type theirJp;
    haptic_wrist::jp_type theirWristJp;

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);
    haptic_wrist::HapticWrist* hw;
    MagnumGripper* gripper;
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF + 3> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(30);
    State state;
};
