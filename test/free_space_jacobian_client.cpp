
#include "ros/ros.h"
#include "catheter_kinematics/Jacobian.h"

int main(int argc, char **argv) {
    // Init node
    ros::init(argc, argv, "catheter_free_space_jacobian_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<catheter_kinematics::Jacobian>("catheter_free_space_jacobian");

    // Put array in service
    catheter_kinematics::Jacobian srv;
    const int dof = 8;
    const int currents_size = 6;
    double joint_angles[dof] = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3};
    double currents[currents_size] = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3};  // For debugging

    std::copy(joint_angles, joint_angles + dof, std::back_inserter(srv.request.joint_angles));
    std::copy(currents, currents + currents_size, std::back_inserter(srv.request.currents));

    if (client.call(srv)) {
        ROS_INFO("Jacobian = ");
        for (int i = 0; i < srv.response.jacobian.data.size(); i++) {
            ROS_INFO("%f" , srv.response.jacobian.data[i]);
        }
    }

    else {
        ROS_ERROR("Something went wrong...");
        return 1;
    }

    return 0;
}

