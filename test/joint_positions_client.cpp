//
// Created by tipakorng on 5/19/16.
//

#include "ros/ros.h"
#include <catheter_kinematics/JointPositions.h>

int main(int argc, char **argv) {
    // Init node
    ros::init(argc, argv, "catheter_forward_kinematics_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<catheter_kinematics::JointPositions>("catheter_joint_positions");

    // Put array in service
    catheter_kinematics::JointPositions srv;
    const int joint_angles_size = 8;
    srv.request.joint_angles.insert(srv.request.joint_angles.end(), joint_angles_size, 1e-3);

    if (client.call(srv)) {
        ROS_INFO_STREAM(srv.response);
    }

    else {
        ROS_ERROR("Something went wrong...");
        return 1;
    }

    return 0;
}
