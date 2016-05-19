cmake_minimum_required(VERSION 2.8.3)
project(catheter_kinematics)

# Configure CMake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslib rospy std_msgs geometry_msgs message_generation)
find_package(MATLAB REQUIRED)
find_package(rostest REQUIRED)

# If Matlab is not found, raise error and quit
if(NOT MATLAB_FOUND)
    message(SEND_ERROR "MATLAB not found, nothing will be built.")
endif()

# Declare ROS messages and services
add_service_files(DIRECTORY srv FILES ForwardKinematics.srv Jacobian.srv JointPositions.srv)

# Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# Declare a catkin package
catkin_package()

# Set include directories
include_directories(${catkin_INCLUDE_DIRS} ${Matlab_INCLUDE_DIRS})

# Add executables
#add_executable(catheter_forward_kinematics_server src/forward_kinematics_server.cpp)
#target_link_libraries(catheter_forward_kinematics_server ${catkin_LIBRARIES} ${MATLAB_LIBRARIES})
#add_dependencies(catheter_forward_kinematics_server catheter_kinematics_gencpp)

add_executable(catheter_forward_kinematics_client test/forward_kinematics_client.cpp)
target_link_libraries(catheter_forward_kinematics_client ${catkin_LIBRARIES})
add_dependencies(catheter_forward_kinematics_client catheter_kinematics_gencpp)
#
#add_executable(catheter_free_space_jacobian_server src/free_space_jacobian_server.cpp)
#target_link_libraries(catheter_free_space_jacobian_server ${catkin_LIBRARIES} ${MATLAB_LIBRARIES})
#add_dependencies(catheter_free_space_jacobian_server catheter_kinematics_gencpp)

add_executable(catheter_free_space_jacobian_client test/free_space_jacobian_client.cpp)
target_link_libraries(catheter_free_space_jacobian_client ${catkin_LIBRARIES} ${MATLAB_LIBRARIES})
add_dependencies(catheter_free_space_jacobian_client catheter_kinematics_gencpp)

add_library(catheter_kinematics_lib src/catheter_kinematics_lib.cpp)
target_link_libraries(catheter_kinematics_lib ${catkin_LIBRARIES} ${MATLAB_LIBRARIES})
add_dependencies(catheter_kinematics_lib catheter_kinematics_gencpp)

add_executable(catheter_kinematics_server src/catheter_kinematics_server.cpp)
target_link_libraries(catheter_kinematics_server ${catkin_LIBRARIES} catheter_kinematics_lib)
add_dependencies(catheter_kinematics_server catheter_kinematics_gencpp)

# Add tests
catkin_add_gtest(test_catheter_kinematics_lib test/test_catheter_kinematics_lib.cpp)
target_link_libraries(test_catheter_kinematics_lib catheter_kinematics_lib)
