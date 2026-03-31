#include "haptic_wrist/haptic_wrist.h"
#include "gripper/magnum_opus/magnum_gripper.h"
#include <iostream>
#include <thread>
#include <boost/optional.hpp>

using namespace gripper::magnum_opus;

int main(int argc, char** argv) {
    haptic_wrist::HapticWrist hw;
    MagnumGripper gripper;
    std::cout << "starting gripper and handle test" << std::endl;

    if (!gripper.initialize()) {
        std::cerr << "ERROR: Failed to initialize Magnum Gripper." << std::endl;
        return -1;
    }

    const int trigger_rest_pos = 0.25;
    float target_velocity = 0.1;
    while (true) {
        if (boost::optional<haptic_wrist::handle_type> opt_handle = hw.getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle; 
            float trigger = static_cast<float>(handle[3]);

            // pushing trigger closes gripper
            if (trigger > 0.25) {
                gripper.setVelocity(target_velocity);
            } else {
                gripper.setVelocity(-target_velocity);
            }

            std::cout << trigger << std::endl;
        }

        gripper.controlLoopCallback();
        GripperState state = gripper.getLatestState();
        
        std::cout << "\rPos: " << state.position << " | Trq: " << state.torque << "    " << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gripper.shutdown();
    hw.stop();
    
    return 0;
}
