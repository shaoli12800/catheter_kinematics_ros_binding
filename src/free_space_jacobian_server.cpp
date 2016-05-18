//
// Created by tipakorng on 5/17/16.
//

#include "ros/ros.h"
#include "ros/package.h"
#include "engine.h"
#include "catheter_kinematics/Jacobian.h"

engine *ep_;

bool free_space_jacobian(catheter_kinematics::Jacobian::Request &request,
                         catheter_kinematics::Jacobian::Response &response) {
    // Define dimensions of Jacobian
    unsigned int currentsDim = static_cast<int>(request.currents.size());  // This is okay since we do not plan on having more than 65536 coils
    unsigned int tipPositionDim = 3;
    // Put joint angles and currents in Matlab workspace
    mxArray *jointAnglesMxArray = mxCreateDoubleMatrix(request.jointAngles.size(), 1, mxREAL);
    mxArray *currentsMxArray = mxCreateDoubleMatrix(request.currents.size(), 1, mxREAL);
    std::copy(request.jointAngles.begin(), request.jointAngles.end(), mxGetPr(jointAnglesMxArray));
    std::copy(request.currents.begin(), request.currents.end(), mxGetPr(currentsMxArray));
    engPutVariable(ep_, "jointAngles", currentsMxArray);
    engPutVariable(ep_, "currents", currentsMxArray);
    // Calculate Jacobian
    engEvalString(ep_, "jacobian = free_space_jacobian(jointAngles, currents)");
    // Read Jacobian from Matlab workspace
    mxArray* jacobianMxArray = engGetVariable(ep_, "jacobian");
    // Put Jacobian in response
    response.jacobian.data.insert(response.jacobian.data.end(), mxGetPr(jacobianMxArray), mxGetPr(jacobianMxArray) + currentsDim * tipPositionDim);
    response.jacobian.layout.dim[0].label = "rows";
    response.jacobian.layout.dim[0].size = tipPositionDim;
    response.jacobian.layout.dim[0].stride = currentsDim * tipPositionDim;
    response.jacobian.layout.dim[1].label = "columns";
    response.jacobian.layout.dim[1].size = currentsDim;
    response.jacobian.layout.dim[1].stride = currentsDim;

    return true;
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "catheter_free_space_jacobian_server");
    ros::NodeHandle nh;

    // Start Matlab engine
    ep_ = engOpen("\0");
    std::string changeDir = "cd " + ros::package::getPath("catheter_kinematics") + "/matlab";
    engEvalString(ep_, changeDir.c_str());
    engEvalString(ep_, "run('startup.m')");

    // Advertise service
    ros::ServiceServer service = nh.advertiseService("catheter_free_space_jacobian", free_space_jacobian);
    ROS_INFO("Catheter free-space jacobian server initialized");
    ros::spin();
}
