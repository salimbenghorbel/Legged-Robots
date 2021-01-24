clear all
close all
clc
%%
options = odeset('RelTol', 1e-5);
multiple_runs = false;
pause on;
f = @f_dyn;
nx = 10;
ny = 10;
nu = 4; % previously 2
Ts = 0.05; % previously 0.05
p = 30; % 10 -> 15
f_optim = @(x,u) RK4_f(x,u,Ts,f);
n_steps = 50; % 20

ctrl = nlmpc(nx,ny,nu);
ctrl.Ts = Ts;
ctrl.PredictionHorizon = p;
ctrl.ControlHorizon = p;
ctrl.Optimization.ReplaceStandardCost = true;
ctrl.Optimization.CustomCostFcn = @(X,U,e,data) cost_function(X,U,e,data);
ctrl.Model.StateFcn = f;
ctrl.Model.IsContinuousTime = true;
ctrl.Optimization.CustomIneqConFcn =  @(X,U,e,data) constraints(X,U,e,data);

%% set constraint
max_angle = pi/4;
max_posture = pi/2;
max_input = 2;
length_rate = 10/Ts;
torque_rate = max_input/Ts;
ctrl.ManipulatedVariables(1).Min = -max_input;
ctrl.ManipulatedVariables(1).Max = max_input;
ctrl.ManipulatedVariables(2).Min = -max_input;
ctrl.ManipulatedVariables(2).Max = max_input;
ctrl.ManipulatedVariables(3).Min = 0;
ctrl.ManipulatedVariables(3).Max = 1;
ctrl.ManipulatedVariables(4).Min = 0;
ctrl.ManipulatedVariables(4).Max = 1;

ctrl.ManipulatedVariables(1).RateMin = -torque_rate;
ctrl.ManipulatedVariables(1).RateMax = torque_rate;
ctrl.ManipulatedVariables(2).RateMin = -torque_rate;
ctrl.ManipulatedVariables(2).RateMax = torque_rate;


ctrl.ManipulatedVariables(3).RateMin = -length_rate;
ctrl.ManipulatedVariables(3).RateMax = length_rate;
ctrl.ManipulatedVariables(4).RateMin = -length_rate;
ctrl.ManipulatedVariables(4).RateMax = length_rate;


ctrl.States(3).Min = -max_angle;
ctrl.States(3).Max = max_angle;
ctrl.States(4).Min = -max_angle;
ctrl.States(4).Max = max_angle;
ctrl.States(5).Min = -max_posture;
ctrl.States(5).Max = max_posture;
ctrl.States(1).Min = -1;
ctrl.States(6).Min = 0;
%ctrl.States(3).Max = pi/2;

step_init = round(0.5/Ts);

q0 = [0; 0.6; pi/8; -pi/7; 0]; %0.46
dq0 = [0; 0; 0; 0; 0];
x0 = [q0;dq0];
u0 = [0 0 1 1]';%zeros(nu,1);
validateFcns(ctrl,x0,u0);
X = zeros(nx,step_init+n_steps+1);
X(:,1) = x0;
U = zeros(nu,step_init+n_steps); %n_steps+1
U(:,1) = u0;
%%
if multiple_runs
    for k=1:step_init
        [t,y] = ode45(@(t,y)f_dyn(y,u0),[0;Ts/2;Ts],X(:,k),options);
        X(:,k+1) = y(2,:)';
        %     X(:,k+1) = f_optim(X(:,k), U(:,k));
        U(:,k+1) = u0;
    end
    %%
    % info_list = [];
    for k = step_init:step_init+n_steps
        [mv,~,info] = nlmpcmove(ctrl,X(:,k),U(:,k));
        X(:,k+1) = info.Xopt(2,:)';
        %     U(:,k) = info.MVopt(2,:)'; %2:0
        U(:,k+1) = mv; %2:0
        %     info_list = [info_list, info];
    end
else
    [~,~,info] = nlmpcmove(ctrl,x0,u0);
    X = info.Xopt';
    U = info.MVopt';
end
%% Visualize

close all
for k = 1:length(X)
    q = X(1:5,k);
    % visualize :
    l1c = U(3,k);
    l2c = U(4,k);
    visualize_leg_control(q,l1c,l2c);
    hold off
    pause(0.1);
end

%% Plot

figure(2)
subplot(121)
hold on
plot(U(1,:));
plot(U(2,:));
subplot(122);
hold on
plot(U(3,:));
plot(U(4,:));

%% leg values
[~, ~, ~, l1, l2, ~, ~] = set_parameters;
length_ratio_reduction = 0.2;
l1c = l1*(1 + length_ratio_reduction*(U (3,:)-1));
l2c = l2*(1 + length_ratio_reduction*(U (4,:)-1));

zl1 = zeros(size(l1c));
zl2 = zeros(size(l2c));

for k =1:length(l1c)
    zl1(k) = eval_zl1(X (1:5,k), l1c(k));
    zl2(k) = eval_zl2(X (1:5,k), l2c(k));
end