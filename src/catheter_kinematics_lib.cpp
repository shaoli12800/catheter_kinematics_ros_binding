//
// Created by tipakorng on 5/18/16.
//

#include "catheter_kinematics_lib.h"

CatheterKinematics::CatheterKinematics(ros::NodeHandle &node_handle) : node_handle_(node_handle) {
    ROS_INFO("Initializing services...");
    forward_kinematics_ = node_handle_.advertiseService("catheter_forward_kinematics",
                                                        &CatheterKinematics::forward_kinematics_callback, this);
    free_space_jacobian_ = node_handle_.advertiseService("catheter_free_space_jacobian",
                                                         &CatheterKinematics::free_space_jacobian_callback, this);
    ROS_INFO("Services initialized");
    ROS_INFO("Initializing Matlab engine...");
    engine_pointer_ = engOpen("\0");
    std::string changeDir = "cd " + ros::package::getPath("catheter_kinematics") + "/matlab";
    engEvalString(engine_pointer_, changeDir.c_str());
    engEvalString(engine_pointer_, "run('startup.m')");
    ROS_INFO("Matlab initialized");
    ROS_INFO("Catheter kinematics server is ready");
}

CatheterKinematics::~CatheterKinematics() { }

void CatheterKinematics::forward_kinematics(const std::vector<float> &currents, std::vector<float> &joint_angles) {
    mxArray *currents_mx = mxCreateDoubleMatrix(currents.size(), 1, mxREAL);
    std::copy(currents.begin(), currents.end(), mxGetPr(currents_mx));
    engPutVariable(engine_pointer_, "currents", currents_mx);
    // Calculate quasi-static configuration
    engEvalString(engine_pointer_, "[jointAngles, dof] = quasistatic_conf(currents)");
    // Get answer
    mxArray *dof_mx = engGetVariable(engine_pointer_, "dof");
    mxArray *joint_angles_mx = engGetVariable(engine_pointer_, "jointAngles");
    // Put answer in its container
    int size = static_cast<int>(*reinterpret_cast<double*>(mxGetData(dof_mx)));
    joint_angles.insert(joint_angles.end(), mxGetPr(joint_angles_mx), mxGetPr(joint_angles_mx)+size);
    // Destroy mxArrays
    mxDestroyArray(currents_mx);
    mxDestroyArray(dof_mx);
    mxDestroyArray(joint_angles_mx);
}

void CatheterKinematics::free_space_jacobian(const std::vector<float> &joint_angles, const std::vector<float> &currents,
                                             std::vector<float> &jacobian) {

    // Define dimensions of Jacobian
    int num_rows = 3;
    int num_cols = static_cast<int>(currents.size());  // This is okay since we do not plan on having more than 65536 coils
    // Put joint angles and currents in Matlab workspace
    mxArray *joint_angles_mx = mxCreateDoubleMatrix(joint_angles.size(), 1, mxREAL);
    mxArray *currents_mx = mxCreateDoubleMatrix(currents.size(), 1, mxREAL);
    std::copy(joint_angles.begin(), joint_angles.end(), mxGetPr(joint_angles_mx));
    std::copy(currents.begin(), currents.end(), mxGetPr(currents_mx));
    engPutVariable(engine_pointer_, "joint_angles", joint_angles_mx);
    engPutVariable(engine_pointer_, "currents", currents_mx);
    // Calculate Jacobian
    engEvalString(engine_pointer_, "jacobian = free_space_jacobian(joint_angles, currents)");
    // Read Jacobian from Matlab workspace
    mxArray* jacobian_mx = engGetVariable(engine_pointer_, "jacobian");
    jacobian.insert(jacobian.end(), mxGetPr(jacobian_mx), mxGetPr(jacobian_mx) + num_cols * num_rows);
    // Delete mxArrays
    mxDestroyArray(joint_angles_mx);
    mxDestroyArray(currents_mx);
    mxDestroyArray(jacobian_mx);
}

bool CatheterKinematics::forward_kinematics_callback(catheter_kinematics::ForwardKinematics::Request &request,
                                                     catheter_kinematics::ForwardKinematics::Response &response) {
    forward_kinematics(request.currents, response.joint_angles);
    return true;
}

bool CatheterKinematics::free_space_jacobian_callback(catheter_kinematics::Jacobian::Request &request,
                                                      catheter_kinematics::Jacobian::Response &response) {

    // Define dimensions of Jacobian
    unsigned int currents_size = static_cast<unsigned int>(request.currents.size());  // This is okay since we do not plan on having more than 65536 coils
    unsigned int tip_position_size = 3;
    // Put Jacobian in response
    free_space_jacobian(request.joint_angles, request.currents, response.jacobian.data);
    response.jacobian.layout.data_offset = 0;
    response.jacobian.layout.dim.resize(2);
    response.jacobian.layout.dim[0].label = "rows";
    response.jacobian.layout.dim[0].size = currents_size;
    response.jacobian.layout.dim[0].stride = currents_size * tip_position_size;
    response.jacobian.layout.dim[1].label = "columns";
    response.jacobian.layout.dim[1].size = tip_position_size;
    response.jacobian.layout.dim[1].stride = tip_position_size;

    return true;
}

bool CatheterKinematics::joint_positions_callback(catheter_kinematics::JointPositions::Request &request,
                                                  catheter_kinematics::JointPositions::Response response) {
    return true;
}
