function ctrl = ctrl_NMPC(h,mpc_horizon,f)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = mpc_horizon; % MPC horizon

% ---- decision variables ---------
X = opti.variable(6,N+1); % state trajectory variables
U = opti.variable(2, N); % control trajectory (u_1, u_2)

x_h = opti.variable(1,N+1); % hip x
z_h = opti.variable(1,N+1); % hip z
dx_h = opti.variable(1,N+1); % hip xvelocity
dz_h = opti.variable(1,N+1); % hip zvelocity

% x_t = opti.variable(1,N+1); % torso x
% z_t = opti.variable(1,N+1); % torso z
% dx_t = opti.variable(1,N+1); % torso xvelocity
% dz_t = opti.variable(1,N+1); % torso zvelocity

X0  = opti.parameter(6,1); % initial state
REF = opti.parameter(1,1); % reference position [z_h]

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE BELOW %%%%
%%%%%%%%%%%%%%%%%%%%%%%%

% System dynamics
f_quad = @(x,u) RK4_quad(x,u,h,f);



% steady-state variables
z_hs = opti.variable(1,1); % steady-state z_h variable
dz_hs = opti.variable(1,1);
x_hs = opti.variable(1,1); % steady-state x_h variable
dx_hs = opti.variable(1,1);

Xs = opti.variable(6,1); % steady-state X variable
Us = opti.variable(2, 1); % steady-state input


%% perform control action

gain_xs_ref = 1000;
gain_x_xs = 1;
gain_u_us = 1;
gain_u = 1;
gain_x = 1;
gain_v = 1;
gain_d = 100;
sum_minimize = gain_xs_ref * (z_hs-REF)'*(z_hs-REF);
for i = 1:N
    sum_minimize = sum_minimize + gain_u_us * (U(:,i)-Us)'*(U(:,i)-Us);
    sum_minimize = sum_minimize - gain_d * x_h(:,i)' * x_h(:,i);
    sum_minimize = sum_minimize - gain_v * dx_h(:,i)' * dx_h(:,i);
    sum_minimize = sum_minimize + gain_x_xs * (X(:,i)-Xs)' * (X(:,i)-Xs);
end
sum_minimize = sum_minimize - gain_d * x_h(:,N+1)' * x_h(:,N+1);
sum_minimize = sum_minimize - gain_v * dx_h(:,N+1)' * dx_h(:,N+1);
sum_minimize = sum_minimize + gain_x_xs * (X(:,N+1)-Xs)' * (X(:,N+1)-Xs);

opti.minimize(sum_minimize);

% boundary conditions
angle_limit = pi/2;
u_min = -30;
u_max = 30;

% steady-state conditions
            
opti.subject_to(Xs == f_quad(Xs,Us));
opti.subject_to([x_hs, z_hs, dx_hs, dz_hs] == kin_hip(Xs(1:3),X(4:end)));
opti.subject_to( -angle_limit < X(1,:) < angle_limit);
opti.subject_to( -angle_limit < X(2,:) < angle_limit);
opti.subject_to( -angle_limit < X(3,:) < angle_limit);
opti.subject_to(u_min <= Us <= u_max); 

% loop over control intervals
for k=1:N
    opti.subject_to(X(:,k+1) == f_quad(X(:,k), U(:,k)));
%     opti.subject_to([x_t(k), z_t(k), dx_t(k), dz_t(k)] == kin_torso(X(1:3,k),X(4:end,k)));
    opti.subject_to([x_h(k), z_h(k), dx_h(k), dz_h(k)] == kin_hip(X(1:3,k),X(4:end,k)));
end

% angle conditions
opti.subject_to( -angle_limit < X(1,:) < angle_limit);
opti.subject_to( -angle_limit < X(2,:) < angle_limit);
opti.subject_to( -angle_limit < X(3,:) < angle_limit);

% no-falling conditions
opti.subject_to(z_h>0);
% opti.subject_to(z_t>0);

% input conditions
opti.subject_to(u_min <= U <= u_max);  % control is limited

% boundary conditions
opti.subject_to(X(:,1)==X0);

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE ABOVE%%%%
%%%%%%%%%%%%%%%%%%%%%%%%

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end
