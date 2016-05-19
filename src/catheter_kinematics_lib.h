//
// Created by tipakorng on 5/18/16.
//

#ifndef CATHETER_KINEMATICS_CATHETER_KINEMATICS_H
#define CATHETER_KINEMATICS_CATHETER_KINEMATICS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <engine.h>
#include <catheter_kinematics/ForwardKinematics.h>
#include <catheter_kinematics/Jacobian.h>
#include <catheter_kinematics/JointPositions.h>
#include <geometry_msgs/Point32.h>
#include <gtest/gtest_prod.h>

class CatheterKinematics {

public:

    // Constructor
    CatheterKinematics(ros::NodeHandle &nodeHandle);

    // Destructor
    ~CatheterKinematics();

private:

    // Calculate quasi-static joint angles from currents
    void forward_kinematics(const std::vector<float> &currents, std::vector<float> &joint_angles);

    // Calculate free-space Jacobian
    void free_space_jacobian(const std::vector<float> &joint_angles, const std::vector<float> &currents,
                                 std::vector<float> &jacobian);

    // Forward kinematics callback function
    bool forward_kinematics_callback(catheter_kinematics::ForwardKinematics::Request &request,
                                     catheter_kinematics::ForwardKinematics::Response &response);

    // Free-space Jacobian callback function
    bool free_space_jacobian_callback(catheter_kinematics::Jacobian::Request &request,
                                      catheter_kinematics::Jacobian::Response &response);

    // Joint positions callback function
    bool joint_positions_callback(catheter_kinematics::JointPositions::Request &request,
                                  catheter_kinematics::JointPositions::Response);

    // ROS node handle
    ros::NodeHandle node_handle_;

    // Forward kinematics service
    ros::ServiceServer forward_kinematics_;

    // Free space Jacobian service
    ros::ServiceServer free_space_jacobian_;

    // Matlab engine pointer
    engine* engine_pointer_;

    // FRIEND_TEST allows gtest to access private member functions/variables during testing
    FRIEND_TEST(CatheterKinematicsTest, testForwardKinematicsCallback);
    FRIEND_TEST(CatheterKinematicsTest, testFreeSpaceJacobianCallback);
};

#endif //CATHETER_KINEMATICS_CATHETER_KINEMATICS_H
