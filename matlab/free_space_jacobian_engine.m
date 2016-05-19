function [jacobian, num_rows, num_cols] = free_space_jacobian_engine(joint_angles, currents)
% TODO: Add documentation
global PARAM;

jacobian = tip_jacobian(joint_angles, currents, PARAM)';  % C++ arrays are row-major
num_rows = size(jacobian, 1);
num_cols = size(jacobian, 2);
disp(joint_angles);
end