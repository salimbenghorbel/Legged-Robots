clear all
close all
clc
%%
pause on;
f = @f_dyn;
nx = 10;
ny = 10;
nu = 2;
Ts = 0.05;
p = 15;
f_optim = @(x,u) RK4_f(x,u,Ts,f);
n_steps = 30;

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
max_angle = pi/2;
max_input = 30;
ctrl.ManipulatedVariables(1).Min = -max_input;
ctrl.ManipulatedVariables(1).Max = max_input;
ctrl.ManipulatedVariables(2).Min = -max_input;
ctrl.ManipulatedVariables(2).Max = max_input;
ctrl.States(3).Min = -max_angle;
ctrl.States(3).Max = max_angle;
ctrl.States(4).Min = -max_angle;
ctrl.States(4).Max = max_angle;
ctrl.States(1).Min = -1;
%ctrl.States(3).Max = pi/2;


q0 = [0; 0.5; pi/6; -pi/4; 0];
dq0 = [0; 0; 0; 0; 0];
x0 = [q0;dq0]; 
u0 = zeros(nu,1); 
validateFcns(ctrl,x0,u0);
[~,~,info] = nlmpcmove(ctrl,x0,u0);
X = zeros(nx,n_steps+1);
X(:,1) = x0;
U = zeros(nu,n_steps+1);
U(:,1) = u0;
%%
% info_list = [];
% for k = 1:n_steps
%     [~,~,info] = nlmpcmove(ctrl,X(:,k),U(:,k));
%     X(:,k+1) = info.Xopt(2,:)';
%     U(:,k+1) = info.MVopt(2,:)';
%     info_list = [info_list, info];
% end
%%
close all
X = info.Xopt';
for k = 1:length(X)
    q = X(1:5,k);
    % visualize :
    visualize(q);
    hold off
    pause(0.1);
end


    