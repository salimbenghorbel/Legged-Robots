clear all
close all
clc
%%
pause on;
f = @f_dyn_7;
nx = 7;
ny = 7;
nu = 2;
Ts = 0.01;
p = 15;
f_optim = @(x,u) RK4_f(x,u,Ts,f);
n_steps = 300;

multiple_runs = true;

ctrl = nlmpc(nx,ny,nu);
ctrl.Ts = Ts;
ctrl.PredictionHorizon = p;
ctrl.ControlHorizon = p;
ctrl.Optimization.ReplaceStandardCost = true;
ctrl.Optimization.CustomCostFcn = @(X,U,e,data) cost_function(X,U,e,data);
ctrl.Model.StateFcn = f_optim;
ctrl.Model.IsContinuousTime = false;
%ctrl.Optimization.CustomIneqConFcn =  @(X,U,e,data) constraints(X,U,e,data);

%% set constraint
max_angle = pi/4;
max_input = 30;
ctrl.ManipulatedVariables(1).Min = -max_input;
ctrl.ManipulatedVariables(1).Max = max_input;
ctrl.ManipulatedVariables(2).Min = -max_input;
ctrl.ManipulatedVariables(2).Max = max_input;
ctrl.States(1).Min = -max_angle;
ctrl.States(1).Max = max_angle;
ctrl.States(2).Min = -max_angle;
ctrl.States(2).Max = max_angle;
ctrl.States(3).Min = -pi/2;
ctrl.States(3).Max = pi/2;
ctrl.States(7).Min = 0; % force system to go forward


q0 = [pi/6; -pi/3; 0];
dq0 = [0;0;0];
r0 = 0;
x0 = [q0;dq0;r0]; 
u0 = zeros(nu,1); 
validateFcns(ctrl,x0,u0);
X = zeros(nx,n_steps+1);
X(:,1) = x0;
U = zeros(nu,n_steps+1);
U(:,1) = u0;
%%
if multiple_runs 
    for k = 1:n_steps
        [~,~,info] = nlmpcmove(ctrl,X(:,k),U(:,k));
        X(:,k+1) = info.Xopt(2,:)';
        U(:,k+1) = info.MVopt(1,:)';
    end
else
    [~,~,info] = nlmpcmove(ctrl,x0,u0);
    X = info.Xopt';
end
%%
close all
x_h = zeros(1,length(X));
z_h = zeros(1,length(X));
dx_h = zeros(1,length(X));
for k = 1:length(X)
    [x_h(k), z_h(k), dx_h(k), ~] = kin_hip(X(1:3,k),X(4:6,k));
    visualize(X(1:3,k),[x_h(k) + X(7,k);0]);
    hold off
    pause(0.5);
end


    