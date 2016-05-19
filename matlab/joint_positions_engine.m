function [joint_positions, dimension, num_joints] = joint_positions_engine(joint_angles)
% TODO: Add documentation

global PARAM;
dimension = 3;
num_joints = PARAM.catheter.num_joints;
joint_positions = joint_location(joint_angles, PARAM);

end