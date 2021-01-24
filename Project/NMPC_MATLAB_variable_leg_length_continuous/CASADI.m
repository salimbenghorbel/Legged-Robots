clear; close all;
addpath('D:/CASADI')
import casadi.*

h = 0.05;
f = @f_dyn;
f_discrete = @(x,u) RK4_f(x,u,h,f);


%% Ex 3 MPC problem setup

opti = casadi.Opti(); % Optimization problem

N = 10; % MPC horizon

% ---- decision variables ---------
X   = opti.variable(10,N+1); % state trajectory variables
U   = opti.variable(4,N);   % control trajectory (throttle, brake)

X0  = opti.parameter(10,1); % parameter variable for initial state
q0  = X0(1:5); % Initial position and velocity
dq0 = X0(6:10);

q   = X(1:5,:);
dq  = X(6:10,:);

% ---- objective ---------
opti.minimize(...
  -10*X(1,end)  + ... % Max velocity
  0.1*U(1,:)*U(1,:)' + ... % Minimize accel
  0.1*U(2,:)*U(2,:)' )%  + ... % Minimize braking

% ---- multiple shooting --------
for k=1:N % loop over control intervals
    opti.subject_to(X(:,k+1) == f_discrete(X(:,k), U(:,k)));
end
for k=1:N-1
    opti.subject_to(-1 <= (U(:,k+1) - U(:,k))/h <= 1);
end

% ---- path constraints -----------

max_torque = 100;
opti.subject_to(-max_torque <= U(1,:) <= max_torque);  % control is limited
opti.subject_to(-max_torque <= U(2,:) <= max_torque);  % control is limited
opti.subject_to(0 <= U(3,:) <= 1);  % control is limited
opti.subject_to(0 <= U(4,:) <= 1);  % control is limited

% ---- boundary conditions --------
opti.subject_to(q(:,1)==q0);   % use initial position
opti.subject_to(dq(:,1)==dq0); % use initial speed

% Pass parameter values
opti.set_value(q0, [0; 5; pi/6; -pi/6; 0]);
opti.set_value(dq0, [0; 0; 0; 0; 0]);

% ---- Setup solver NLP    ------
ops = struct;
ops.ipopt.print_level = 0;
ops.ipopt.tol = 1e-3;
opti.solver('ipopt', ops);
sol = opti.solve();   % actual solve

%% closed loop simulation

sim.x = [0; 5; pi/6; pi/6; 0; 0; 0; 0; 0; 0]; % Start at position zero with zero speed
sim.u = [0; 0; 1; 1];
sim.t = 0;

figure(2); clf; plot_track(track); hndl = [];

M = 100;

for i=1:M  %% Take one loop around the track
  sim.t(end+1) = sim.t(end) + h; 
  
  % set position and speed variables to their current values
  opti.set_value(X0, sim.x(:,i));
  
  % ---- call the MPC controller ------
  sol = opti.solve();
  u_mpc = sol.value(U(:,1)); % Get the MPC input
  
  % ---- Simulate the system
  sim.u(:,i) = u_mpc;
  sim.x(:,i+1) = f_discrete(sim.x(:,i), u_mpc);
  
  % warm start the next iteration
  opti.set_initial(U, sol.value(U));
  opti.set_initial(X, sol.value(X));
end
