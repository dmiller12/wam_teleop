#include "haptic_wrist/haptic_wrist.h"
#include "gripper/magnum_opus/magnum_gripper.h"
#include <iostream>
#include <thread>
#include <boost/optional.hpp>

using namespace gripper::magnum_opus;

int main(int argc, char** argv) {
    haptic_wrist::HapticWrist hw;
    MagnumGripper gripper;

    if (!gripper.initialize()) {
        std::cerr << "ERROR: Failed to initialize Magnum Gripper." << std::endl;
        return -1;
    }
    while (true) {
        if (boost::optional<haptic_wrist::handle_type> opt_handle = hw.getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle; 

            float target_velocity = static_cast<float>(handle[3]);
            std::cout << target_velocity << std::endl;

            // gripper.setVelocity(target_velocity);
            gripper.controlLoopCallback();

            GripperState state = gripper.getLatestState();
            
            std::cout << "\rTrigger: " << target_velocity 
                      << " | Pos: " << state.position 
                      << " | Trq: " << state.torque << "    " << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gripper.shutdown();
    hw.stop();
    
    return 0;
}
