clear all
close all
clc
%%
pause on;
f = @f_dyn;
nx = 6;
ny = 6;
nu = 2;
Ts = 0.4;
p = 30;
f_optim = @(x,u) RK4_f(x,u,Ts,f);

ctrl = nlmpc(nx,ny,nu);
ctrl.Ts = Ts;
ctrl.PredictionHorizon = p;
ctrl.Optimization.ReplaceStandardCost = true;
ctrl.Optimization.CustomCostFcn = @(X,U,e,data) cost_function(X,U,e,data);
ctrl.Model.StateFcn = f_optim;
% ctrl.Optimization.CustomIneqConFcn =  @(X,U,e,data) constraints(X,U,e,data);
q0 = [pi/6; -pi/3; 0];
dq0 = [0;0;0];
x0 = [q0;dq0]; 
u0 = zeros(nu,1); 
validateFcns(ctrl,x0,u0);
[~,~,info] = nlmpcmove(ctrl,x0,u0);
X = zeros(nx,p+1);
X(:,1) = x0;
U = zeros(nu,p+1);
U(:,1) = u0;

%%
% for k = 1:p
%     [~,~,info] = nlmpcmove(ctrl,X(:,k),U(:,k));
%     X(:,k+1) = info.Xopt(2,:)';
%     U(:,k+1) = info.MVopt(2,:)';
% end
%%
X = info.Xopt';
for k = 1:length(X)
    visualize(X(1:3,k));
    hold off
    pause(0.1);
end


    