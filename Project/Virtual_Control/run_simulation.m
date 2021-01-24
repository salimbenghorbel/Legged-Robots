clc;
clear;
close all;

%% run simulation
q0 = [-pi/9; pi/9; 0];
dq0 = [1.5; 0.7; 0.5];
num_steps = 60;

%% internal perturbation definition
std = 4.3;

default_parameters = control_hyper_parameters();
disturbance_list = normrnd(0,std,1,100);

sln = solve_eqns(q0, dq0, num_steps, default_parameters, disturbance_list);
animate(sln, default_parameters);
% analyse(sln, default_parameters, true);