/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>
#include "ros/ros.h"

#include "leader.h"
#include "tool_frame_cb.h"
#include "background_state_publisher.h"

using namespace barrett;
using detail::waitForEnter;

bool validate_args(int argc, char **argv) {
    if (argc != 3 && argc != 4) {
        printf("Usage: %s <remoteHost> <recPort> [sendPort]\n", argv[0]);
        printf("    sendPort: If not provided sendPort = recPort -1\n");

        return false;
    }

    return true;
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 4) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.5;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 2.7;

    } else {
        printf("Error: Only 4 DOF wam supported\n");
        return false;
    }

    int rec_port = atoi(argv[2]);
    int send_port;
    if (argc == 3) {
        send_port = rec_port - 1;
    } else {
        send_port = atoi(argv[3]);
    }
    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(true);
    hw.run();


    ros::init(argc, argv, "leader");
    BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam, &hw);

    ToolFrameCb toolframeCb(&hw);
    barrett::systems::connect(wam.toolPose.output, toolframeCb.input);
    pm.getExecutionManager()->startManaging(toolframeCb);

    Leader<DOF> leader(pm.getExecutionManager(), &hw, argv[1], rec_port, send_port);
    systems::connect(wam.jpOutput, leader.wamJPIn);

    wam.gravityCompensate();

    std::string line;
    v_type gainTmp;

    bool going = true;

    while (going) {
        printf(">>> ");
        std::getline(std::cin, line);

        switch (line[0]) {
        case 'l':
            if (leader.isLinked()) {
                leader.unlink();
            } else {
                wam.moveTo(SYNC_POS);
                hw.moveTo({0, 0, 0});

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                leader.tryLink();
                wam.trackReferenceSignal(leader.wamJPOutput);

                btsleep(0.1); // wait an execution cycle or two
                if (leader.isLinked()) {
                    printf("Linked with remote WAM.\n");
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }
            }

            break;

        case 't':
            size_t jointIndex;
            {
                size_t jointNumber;
                std::cout << "\tJoint: ";
                std::cin >> jointNumber;
                jointIndex = jointNumber - 1;

                if (jointIndex >= DOF) {
                    std::cout << "\tBad joint number: " << jointNumber;
                    break;
                }
            }

            char gainId;
            std::cout << "\tGain identifier (p, i, or d): ";
            std::cin >> line;
            gainId = line[0];

            std::cout << "\tCurrent value: ";
            switch (gainId) {
            case 'p':
                gainTmp = wam.jpController.getKp();
                break;
            case 'i':
                gainTmp = wam.jpController.getKi();
                break;
            case 'd':
                gainTmp = wam.jpController.getKd();
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }
            std::cout << gainTmp[jointIndex] << std::endl;

            std::cout << "\tNew value: ";
            std::cin >> gainTmp[jointIndex];
            switch (gainId) {
            case 'p':
                wam.jpController.setKp(gainTmp);
                break;
            case 'i':
                wam.jpController.setKi(gainTmp);
                break;
            case 'd':
                wam.jpController.setKd(gainTmp);
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }

            break;
        case 'x':
            going = false;
            break;

        default:
            printf("\n");
            printf("    'l' to toggle linking with other WAM\n");
            printf("    't' to tune control gains\n");
            printf("    'x' to exit\n");

            break;
        }
    }


    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    hw.stop();

    return 0;
}

