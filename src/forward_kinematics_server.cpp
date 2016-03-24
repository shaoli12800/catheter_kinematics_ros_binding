
#include "ros/ros.h"
#include "engine.h"
#include "catheter_prb_model/ForwardKinematics.h"

engine *ep_;

bool forward_kinematics(catheter_prb_model::ForwardKinematics::Request &request,
                        catheter_prb_model::ForwardKinematics::Response &response) {
    // Put control in Matlab workspace
    mxArray *controlMxArray = mxCreateDoubleMatrix(request.control.size(), 1, mxREAL);
    std::copy(request.control.begin(), request.control.end(), mxGetPr(controlMxArray));
    engPutVariable(ep_, "control", controlMxArray);
    // Calculate quasi-static configuration
    engEvalString(ep_, "[jointAngles, dof] = quasistatic_conf(control)");
    // Get answer
    mxArray *dofMxArray = engGetVariable(ep_, "dof");
    mxArray *jointAnglesMxArray = engGetVariable(ep_, "jointAngles");
    int dof = static_cast<int>(*reinterpret_cast<double*>(mxGetData(dofMxArray)));
    response.dof = dof;
    response.jointAngles.insert(response.jointAngles.end(), mxGetPr(jointAnglesMxArray), mxGetPr(jointAnglesMxArray) + dof);
    ROS_INFO("dof = %d", response.dof);
    ROS_INFO("jointAngles = ");

    for (int i = 0; i < dof; i++) {
        ROS_INFO("%f", response.jointAngles[i]);
    }
    return true;
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "catheter_forward_kinematics_server");
    ros::NodeHandle nh;

    // Start Matlab engine
    ep_ = engOpen("\0");
    engEvalString(ep_, "cd ~/catkin_ws/src/catheter_prb_model/matlab");  // TODO: Make path finding dynamic
    engEvalString(ep_, "run('startup.m')");  // TODO: Make path finding dynamic

    // Advertise service
    ros::ServiceServer service = nh.advertiseService("catheter_forward_kinematics", forward_kinematics);
    ROS_INFO("Catheter forward kinematics server initialized");
    ros::spin();
}
