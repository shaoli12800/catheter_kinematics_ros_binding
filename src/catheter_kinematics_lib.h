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
#include <gtest/gtest_prod.h>

class CatheterKinematics {

public:

    CatheterKinematics(ros::NodeHandle &nodeHandle);

    ~CatheterKinematics();

private:

    void free_space_jacobian(const std::vector<float> &joint_angles, const std::vector<float> &currents,
                                 std::vector<float> &jacobian);

    void forward_kinematics(const std::vector<float> &currents, std::vector<float> &joint_angles);

    bool free_space_jacobian_callback(catheter_kinematics::Jacobian::Request &request,
                                      catheter_kinematics::Jacobian::Response &response);

    bool forward_kinematics_callback(catheter_kinematics::ForwardKinematics::Request &request,
                                     catheter_kinematics::ForwardKinematics::Response &response);

    ros::NodeHandle node_handle_;

    ros::ServiceServer forward_kinematics_;

    ros::ServiceServer free_space_jacobian_;

    engine* engine_pointer_;

    FRIEND_TEST(CatheterKinematicsTest, testForwardKinematics);

    FRIEND_TEST(CatheterKinematicsTest, testFreeSpaceJacobian);

    FRIEND_TEST(CatheterKinematicsTest, testForwardKinematicsCallback);

    FRIEND_TEST(CatheterKinematicsTest, testFreeSpaceJacobianCallback);
};

#endif //CATHETER_KINEMATICS_CATHETER_KINEMATICS_H
