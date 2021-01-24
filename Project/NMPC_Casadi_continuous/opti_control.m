
%%
clc
clear all
close all


%%
h = 0.01; % timestep
mpc_horizon = 15; % nb mpc horizon steps
tmax = 2;
q0 = [0; 0.5; pi/6; -pi/4; 0];
dq0 = rand(5,1)/1000;
x0 = [q0;dq0];
num_steps = 200;
ref = 0.6;
CTRL = ctrl_NMPC(h,mpc_horizon,@f_dyn);


sim.t = 0;
sim.x = x0;
for i = 1:ceil(tmax/h)
    [sim(i).u, opti] = CTRL(sim(i).x);
    sim(i+1).x = RK4_f(sim(i).x,sim(i).u,h,@f_dyn);
    sim(i+1).t = sim(i).t + h;
end

