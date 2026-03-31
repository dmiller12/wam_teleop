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

    const double trigger_rest_pos = 0.25;
    float target_velocity = 0.3;
    const float torque_scaling = 3;
    const float minStiffness = 0.3;  // Base spring force for moving through empty air
    const float maxStiffness = 2.0;  // Max pushback when gripper is stalled/crushing
    float trigger = 0.0;
    bool bumper_pressed = false;

    while (true) {
        if (boost::optional<haptic_wrist::handle_type> opt_handle = hw.getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle; 
            trigger = static_cast<float>(handle[3]);
            bumper_pressed = static_cast<int>(handle[2]) == 1;

            // pushing trigger closes gripper
            // pushing bumper opens gripper
            if (trigger > trigger_rest_pos) {
                gripper.setVelocity(target_velocity * trigger);
            } else if (bumper_pressed) {
                gripper.setVelocity(-target_velocity);
            } else {
                gripper.setVelocity(0.0f);
            }
        }

        gripper.controlLoopCallback();
        GripperState state = gripper.getLatestState();
        
        // std::cout << "\rPos: " << state.position << " | Trq: " << state.torque << "    " << std::flush;
        if (state.torque > minStiffness) {
            float dynamicStiffness = state.torque * torque_scaling * (maxStiffness - minStiffness) + minStiffness;
            uint8_t haptics = trigger * 255.0 * dynamicStiffness;
            if (haptics > 230) haptics = 230;

            hw.setTriggerHaptics(haptics);
        } else {
            hw.setTriggerHaptics(0); 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gripper.shutdown();
    hw.stop();
    
    return 0;
}
