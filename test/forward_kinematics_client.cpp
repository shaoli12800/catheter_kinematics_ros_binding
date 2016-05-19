
#include "ros/ros.h"
#include "catheter_kinematics/ForwardKinematics.h"

int main(int argc, char **argv) {
    // Init node
    ros::init(argc, argv, "catheter_forward_kinematics_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<catheter_kinematics::ForwardKinematics>("catheter_forward_kinematics");

    // Put array in service
    catheter_kinematics::ForwardKinematics srv;
    const int controls_size = 6;
    srv.request.currents.insert(srv.request.currents.end(), controls_size, 1e-3);

    if (client.call(srv)) {
        ROS_INFO_STREAM(srv.response);
    }

    else {
        ROS_ERROR("Something went wrong...");
        return 1;
    }

    return 0;
}
