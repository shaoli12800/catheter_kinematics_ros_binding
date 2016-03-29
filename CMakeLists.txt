cmake_minimum_required(VERSION 2.8.3)
project(catheter_prb_model)

# Configure CMake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslib rospy std_msgs message_generation)
find_package(MATLAB REQUIRED)

# If Matlab is not found, raise error and quit
if(NOT MATLAB_FOUND)
    message(SEND_ERROR "MATLAB not found, nothing will be built.")
endif()

# Declare ROS messages and services
add_service_files(DIRECTORY srv FILES ForwardKinematics.srv)

# Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Declare a catkin package
catkin_package()

# Set include directories
include_directories(${catkin_INCLUDE_DIRS} ${Matlab_INCLUDE_DIRS})

# Add executables
add_executable(catheter_forward_kinematics_server src/forward_kinematics_server.cpp)
target_link_libraries(catheter_forward_kinematics_server ${catkin_LIBRARIES} ${MATLAB_LIBRARIES})
add_dependencies(catheter_forward_kinematics_server catheter_prb_model_gencpp)

add_executable(catheter_forward_kinematics_client src/forward_kinematics_client.cpp)
target_link_libraries(catheter_forward_kinematics_client ${catkin_LIBRARIES})
add_dependencies(catheter_forward_kinematics_client catheter_prb_model_gencpp)
