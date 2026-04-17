#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
#include <atomic>
#include <thread>
#include <chrono>
#include "gripper/gecko/gecko_gripper.h"


#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

using namespace gripper::gecko;

template <size_t DOF>
class Follower : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Output<jp_type> wamJPOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Follower(barrett::systems::ExecutionManager* em, GeckoGripper* gripper, const std::string& remoteHost, int rec_port = 5554,
                      int send_port = 5555, const std::string& sysName = "Follower")
        : System(sysName)
        , theirJp(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamJPOutput(this, &jpOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , gripper(gripper)
        , target_gripper_vel(0.0f)
        , current_gripper_torque(0.0f)
        , io_running(false)
        , state(State::INIT) {

        if (em != NULL) {
            em->startManaging(*this);
        }

        io_running.store(true);
        io_thread = std::thread(&Follower::pollGripper, this);
    }

    virtual ~Follower() {
        io_running.store(false);
        if (io_thread.joinable()) {
            io_thread.join();
        }
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
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    jp_type commandJp;

    using ReceivedData = typename UDPHandler<DOF>::ReceivedData;

    virtual void operate() {

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        sendJpMsg << wamJP;

        udp_handler.send(sendJpMsg, current_gripper_torque.load());

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {
            theirJp = received_data->jp;
            target_gripper_vel.store(received_data->gripper_data);
        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        switch (state) {
            case State::INIT:
                commandJp = wamJP;
                resetJ7Hybrid();
                jpOutputValue->setData(&commandJp);
                break;
            case State::LINKED:
                commandJp = theirJp;
                applyJoint7Hybrid();
                jpOutputValue->setData(&commandJp);
                break;
            case State::UNLINKED:
                commandJp = wamJP;
                resetJ7Hybrid();
                jpOutputValue->setData(&commandJp);
                break;
        }
    }

    jp_type theirJp;

    void pollGripper() {
        while (io_running.load()) {
            gripper->setVelocity(target_gripper_vel.load());
            gripper->controlLoopCallback();
            
            GripperState gripper_state = gripper->getLatestState();
            current_gripper_torque.store(gripper_state.torque);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        gripper->setVelocity(0.0f);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Follower);
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(50);
    State state;

    GeckoGripper* gripper;
    std::thread io_thread;
    std::atomic<bool> io_running;
    std::atomic<float> target_gripper_vel;
    std::atomic<float> current_gripper_torque;

    static constexpr size_t J7_INDEX = 6;
    const double j7_joy_deadband = 0.05;
    const double j7_max_velocity_rad_s = 1.0;
    bool j7_initialized = false;
    bool j7_joystick_active = false;
    double j7_command_pos = 0.0;
    std::chrono::steady_clock::time_point j7_last_update;

    void resetJ7Hybrid() {
        j7_initialized = false;
        j7_joystick_active = false;
    }

    void applyJoint7Hybrid() {
        if constexpr (DOF <= J7_INDEX) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!j7_initialized) {
            j7_command_pos = wamJP(J7_INDEX);
            j7_last_update = now;
            j7_initialized = true;
        }

        double dt = std::chrono::duration<double>(now - j7_last_update).count();
        j7_last_update = now;
        if (dt < 0.0) {
            dt = 0.0;
        } else if (dt > 0.1) {
            dt = 0.1;
        }

        const double joy_cmd = theirJp(J7_INDEX);
        const bool active = std::abs(joy_cmd) > j7_joy_deadband;

        if (active) {
            const double desired_vel = j7_max_velocity_rad_s * joy_cmd;
            j7_command_pos += desired_vel * dt;
            j7_joystick_active = true;
        } else {
            if (j7_joystick_active) {
                // Latch at the current measured position when returning to center.
                j7_command_pos = wamJP(J7_INDEX);
            }
            j7_joystick_active = false;
        }

        commandJp(J7_INDEX) = j7_command_pos;
    }
};
