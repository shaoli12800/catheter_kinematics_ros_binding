function [jointAngles, dof] = forward_kinematics_engine(control)
% TODO: Add documentation
global PARAM;
dofPerJoint = 2;
disturbance = zeros(3, 1);
dof = dofPerJoint*PARAM.catheter.num_joints;

while true
    initGuess = 0.1*randn(dof, 1);
    
    try
        jointAngles = equilibrium_conf(initGuess, control, disturbance, PARAM);
        disp('Equilibrium configuration found!');
        break
    catch
        disp('Cannot find an equilibrium configuration, retrying with new initial guess...');
    end
end
end