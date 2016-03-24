% Initialize PRB model

% Add PRB model and common source code to path
addpath('./catheter_kinematics_matlab/src/prb_model/src');
addpath('./catheter_kinematics_matlab/src/prb_model/parameters');
addpath(genpath('./catheter_kinematics_matlab/src/common'));
% Initialize global parameter variable
global PARAM;
PARAM = init_param('parameters.m');
% Initialize global state equation handle
global STATE_EQN;
STATE_EQN = @(t, q, q_dot, u, w)state_eqn_constr(t, q, q_dot, u, w, PARAM);