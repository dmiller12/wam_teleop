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
#include <barrett/systems/kinematics_base.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "follower.h"
#include "background_state_publisher.h"
#include "orientation_controller.h"
#include "tool_orientation.h"
#include "print_orientation.h"

using namespace barrett;
using detail::waitForEnter;

void printUsage(const std::string& programName, const std::string& remoteHost, int recPort, int sendPort) {
    std::cout << "Usage: " << programName << " [remoteHost] [recPort] [sendPort]" << std::endl;
    std::cout << "       Defaults: remoteHost=" << remoteHost << ", recPort=" << recPort << ", sendPort=" << sendPort
              << std::endl;
    std::cout << "       -h or --help: Display this help message." << std::endl;
}
bool validate_args(int argc, char** argv) {

    if ((argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) || (argc > 4)) {
        printUsage(argv[0], "127.0.0.1", 5554, 5555);
        return 0;
    }

    return true;
}

template <size_t DOF>
typename units::JointTorques<DOF>::type combineTorques(const boost::tuple<typename units::JointTorques<DOF>::type, typename units::JointTorques<3>::type>& t) {

    typename units::JointTorques<DOF>::type out = boost::get<0>(t); 
    out.segment(4, 3) = boost::get<1>(t);
    return out;
}

template <size_t DOF>
typename units::JointPositions<3>::type extractWristPositions(const typename units::JointPositions<DOF>::type& full_vector)
{
    return full_vector.template tail<3>();
}

template <size_t DOF>
typename units::JointVelocities<3>::type extractWristVelocities(const typename units::JointVelocities<DOF>::type& full_vector)
{
    return full_vector.template tail<3>();
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 7) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.5;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 2.7;
        SYNC_POS[4] = 0.0;
        SYNC_POS[5] = 0.0;
        SYNC_POS[6] = 0.0;

    } else {
        printf("Error: 7 DOF supported\n");
        return false;
    }

    std::string remoteHost = "127.0.0.1";
    int rec_port = 5554;
    int send_port = 5555;

    if (argc >= 2) {
        remoteHost = std::string(argv[1]);
    }
    if (argc >= 3) {
        rec_port = std::atoi(argv[2]);
    }
    if (argc >= 4) {
        send_port = std::atoi(argv[3]);
    }

    ros::init(argc, argv, "follower");
    BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam);

    Follower<DOF> follower(pm.getExecutionManager(), remoteHost, rec_port, send_port);
    systems::connect(wam.jpOutput, follower.wamJPIn);
    systems::connect(wam.jvOutput, follower.wamJVIn);

    systems::KinematicsBase<3> kinematicsWrist(pm.getConfig().lookup("wam7w")["kinematics_wrist"]);
    systems::Callback<jp_type, units::JointPositions<3>::type> wristPositions(extractWristPositions<DOF>);
    systems::Callback<jv_type, units::JointVelocities<3>::type> wristVelocities(extractWristVelocities<DOF>);

    systems::connect(wam.jpOutput, wristPositions.input);
    systems::connect(wam.jvOutput, wristVelocities.input);
    
    systems::connect(wristPositions.output, kinematicsWrist.jpInput);
    systems::connect(wristVelocities.output, kinematicsWrist.jvInput);

    WristOrientationController<3> orientationController;
    orientationController.setKp(4.2);
    orientationController.setKd(0.042);
    systems::connect(kinematicsWrist.kinOutput, orientationController.kinInput);

    ToolOrientation<3> wristOrientation;

    systems::connect(kinematicsWrist.kinOutput, wristOrientation.kinInput);
    systems::connect(wristOrientation.output, orientationController.feedbackInput);
    systems::connect(wristOrientation.output, follower.wamOrientationIn);
    systems::connect(follower.wristOrientationOutput, orientationController.referenceInput);

    // PrintOrientation printLeaderOrientation(pm.getExecutionManager(), "Leader Orientation: ");
    // systems::connect(follower.wristOrientationOutput, printLeaderOrientation.input);

    // PrintOrientation printFollowerOrientation(pm.getExecutionManager(), "Follower Orientation: ");
    // systems::connect(wristOrientation.output, printFollowerOrientation.input);

    systems::TupleGrouper<jt_type, units::JointTorques<3>::type> tg;

    systems::connect(follower.wamJTOutput, tg.template getInput<0>());
    systems::connect(orientationController.controlOutput, tg.template getInput<1>());

    systems::Callback<boost::tuple<jt_type, units::JointTorques<3>::type>, jt_type> torqueCombineCallback(combineTorques<DOF>);

    systems::connect(tg.output, torqueCombineCallback.input);

    // systems::PrintToStream<jt_type> printTorque(pm.getExecutionManager(), "Torque: "); 
    // systems::connect(torqueCombineCallback.output, printTorque.input);

    wam.gravityCompensate();

    std::string line;
    v_type gainTmp;

    bool going = true;

    while (going) {
        printf(">>> ");
        std::getline(std::cin, line);

        switch (line[0]) {
        case 'l':
            if (follower.isLinked()) {
                follower.unlink();
            } else {
                wam.moveTo(SYNC_POS);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                follower.tryLink();
                wam.trackReferenceSignal(torqueCombineCallback.output);

                btsleep(0.1); // wait an execution cycle or two
                if (follower.isLinked()) {
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

    return 0;
}

