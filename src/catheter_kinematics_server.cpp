//
// Created by tipakorng on 5/18/16.
//

#include "catheter_kinematics_lib.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "catheter_kinematics");
    ros::NodeHandle node_handle;
    CatheterKinematics catheter_kinematics(node_handle);
    ros::spin();

    return 0;
}
