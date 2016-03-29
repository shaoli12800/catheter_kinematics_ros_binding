
#include "ros/ros.h"
#include "catheter_prb_model/ForwardKinematics.h"

int main(int argc, char **argv) {
    // Init node
    ros::init(argc, argv, "catheter_forward_kinematics_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<catheter_prb_model::ForwardKinematics>("catheter_forward_kinematics");

    // Put array in service
    catheter_prb_model::ForwardKinematics srv;
    const int controlSize = 6;
    double current[controlSize] = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3};  // For debugging

    std::copy(current, current+controlSize, std::back_inserter(srv.request.control));

    if (client.call(srv)) {
        ROS_INFO("Joint angles = ");
        for (int i = 0; i < srv.response.dof; i++) {
            ROS_INFO("%f" , srv.response.jointAngles[i]);
        }
    }

    else {
        ROS_ERROR("Something went wrong...");
        return 1;
    }

    return 0;
}
