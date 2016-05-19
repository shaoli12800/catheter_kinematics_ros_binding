//
// Created by tipakorng on 5/18/16.
//

//TODO: Finish this test script
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../src/catheter_kinematics_lib.h"
#include <catheter_kinematics/ForwardKinematics.h>
#include <catheter_kinematics/Jacobian.h>

struct CatheterKinematicsTest : testing::Test {

    CatheterKinematics* catheter;

    ros::NodeHandle nh;

    CatheterKinematicsTest() {
        catheter = new CatheterKinematics(nh);
    }

    ~CatheterKinematicsTest() {
        delete catheter;
    }
};

TEST_F(CatheterKinematicsTest, testForwardKinematicsCallback) {
    // TODO: Test should not depend on the exact model of the catheter
    unsigned int dof = 8;
    unsigned int currents_size = 6;
    std::vector<float> currents(currents_size, 1e-3);
    catheter_kinematics::ForwardKinematics srv;
    srv.request.currents = currents;
    catheter->forward_kinematics_callback(srv.request, srv.response);
    ROS_INFO_STREAM(srv.response);
}

TEST_F(CatheterKinematicsTest, testFreeSpaceJacobianCallback) {
    // TODO: Test should not depend on the exact model of the catheter
    unsigned int dof = 8;
    unsigned int currents_size = 6;
    std::vector<float> joint_angles(dof, 1e-3);
    std::vector<float> currents(currents_size, 1e-3);
    catheter_kinematics::Jacobian srv;
    srv.request.joint_angles = joint_angles;
    srv.request.currents = currents;
    catheter->free_space_jacobian_callback(srv.request, srv.response);
    ROS_INFO_STREAM(srv.response);
}

TEST_F(CatheterKinematicsTest, testJointPositionsCallback) {
    // TODO: Test should not depend on the exact model of the catheter
    unsigned int dof = 8;
    std::vector<float> joint_angles(dof, 1e-3);
    catheter_kinematics::JointPositions srv;
    srv.request.joint_angles = joint_angles;
    catheter->joint_positions_callback(srv.request, srv.response);
    ROS_INFO_STREAM(srv.response);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "catheter_kinematics_tester");
    return RUN_ALL_TESTS();
}