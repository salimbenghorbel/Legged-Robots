
%%
clc
clear all
close all


%%
h = 0.01; % timestep
mpc_horizon = 15; % nb mpc horizon steps
tmax = 5;
q0 = [pi/6; -pi/3; 0];
dq0 = [0;0;0];
x0 = [q0;dq0];
num_steps = 200;
ref = 0.485;

CTRL = ctrl_NMPC(h,mpc_horizon,@f_dyn);


sim.t = 0;
sim.x = x0;
for i = 1:ceil(tmax/h)
    sim(i).u = CTRL(sim(i).x,ref);
    sim(i+1).x = RK4_quad_2(sim(i).x,sim(i).u,h,@f_dyn);
end

