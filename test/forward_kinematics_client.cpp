
#include "ros/ros.h"
#include "catheter_kinematics/ForwardKinematics.h"

int main(int argc, char **argv) {
    // Init node
    ros::init(argc, argv, "catheter_forward_kinematics_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<catheter_kinematics::ForwardKinematics>("catheter_forward_kinematics");

    // Put array in service
    catheter_kinematics::ForwardKinematics srv;
    const int controlSize = 6;
    double current[controlSize] = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3};  // For debugging

    std::copy(current, current+controlSize, std::back_inserter(srv.request.currents));

    if (client.call(srv)) {
        ROS_INFO("Joint angles = ");
        for (int i = 0; i < srv.response.joint_angles.size(); i++) {
            ROS_INFO("%f" , srv.response.joint_angles[i]);
        }
    }

    else {
        ROS_ERROR("Something went wrong...");
        return 1;
    }

    return 0;
}
