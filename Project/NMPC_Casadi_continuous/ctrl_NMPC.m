function ctrl = ctrl_NMPC(h,mpc_horizon,f_dyn)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = mpc_horizon; % MPC horizon

% ---- decision variables ---------
X = opti.variable(10,N+1); % state trajectory variables
U = opti.variable(2, N); % control trajectory (u_1, u_2)

X0  = opti.parameter(10,1); % initial state

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE BELOW %%%%
%%%%%%%%%%%%%%%%%%%%%%%%

% System dynamics
f_step = @(x,u) RK4_f(x,u,h,f_dyn);


%% perform control action

gain_u = 2;
gain_d = 100;
gain_v = 100;
gain_posture = 5000;
sum_minimize = 0;
for i = 1:N
%     sum_minimize = sum_minimize + gain_u * U(:,i)'*U(:,i);
%     sum_minimize = sum_minimize - gain_v * X(6,i)'*X(6,i);
    sum_minimize = sum_minimize - gain_d * X(1,i)'*X(1,i);
%     sum_minimize = sum_minimize - gain_posture * X(5,i)'*X(5,i);
end
% sum_minimize = sum_minimize - gain_v * X(6,N+1)'*X(6,N+1);
% sum_minimize = sum_minimize - gain_d * X(1,N+1)'*X(1,N+1);
% sum_minimize = sum_minimize - gain_posture * X(5,N+1)'*X(5,N+1);

opti.minimize(sum_minimize);

% boundary conditions
angle_limit = pi/2;
u_min = -30;
u_max = 30;
            

opti.subject_to( -angle_limit < X(3,:) < angle_limit);
opti.subject_to( -angle_limit < X(4,:) < angle_limit);
opti.subject_to( -angle_limit < X(5,:) < angle_limit);


% loop over control intervals
for k=1:N
    opti.subject_to(X(:,k+1) == f_step(X(:,k), U(:,k)));
end

% no-falling conditions
opti.subject_to(X(2,:) > 0);
% only going forward condition
% opti.subject_to( -1 <= X(1,:));

% input conditions
opti.subject_to(u_min <= U <= u_max);  % control is limited

% boundary conditions
opti.subject_to(X(:,1)==X0);

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE ABOVE%%%%
%%%%%%%%%%%%%%%%%%%%%%%%

ctrl = @(x) eval_ctrl(x, opti, X0, X, U);
end
