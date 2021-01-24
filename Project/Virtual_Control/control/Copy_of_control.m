function u = control(t, q, dq, q0, dq0, step_number, parameters)
% You may call control_hyper_parameters and desired_outputs in this function
% you don't necessarily need to use all the inputs to this control function
[~, ~, ~, l1, l2, l3, ~] = set_parameters();
q1 = q(1);
q2 = q(2);
q3 = q(3);
dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);

x_h = l1*sin(q1);
z_h = l1*cos(q1);
x_swf = l1*sin(q1) - l2*sin(q2);
z_swf = l1*cos(q1) - l2*cos(q2);
x_t = l1*sin(q1) + l3*sin(q3);
z_t = l1*cos(q1) + l3*cos(q3);

dx_h = dq1*l1*cos(q1);
dz_h = -dq1*l1*sin(q1);
dx_swf = dq1*l1*cos(q1) - dq2*l2*cos(q2);
dz_swf = -dq1*l1*sin(q1) + dq2*l2*sin(q2);
dx_t = dq1*l1*cos(q1) + dq3*l3*cos(q3);
dz_t = -dq1*l1*sin(q1) - dq3*l3*sin(q3);

%% Target positions
step = 0.7;

x_hd = step/2;
dx_hd = 0.4; % 0.7 for 1 step per second
x_swfd = step;
z_tr = l1 + l2;

%% Gains
k_hp = 0.1;
k_hv = 0.1;

k_swfp_x = 2; % 100
k_swfv_x = 1; % 1
k_swfp_z = 2;
k_swfv_z = 1;

k_tp = 100; % 1000
k_tv = 100; % 1000
k = 1; % (1) ok
k2 = 0.1; % (0.1) ok

% k_hp = 0.5;
% k_hv = 0.5;
% 
% k_swfp_x = 10;
% k_swfv_x = 0;
% k_swfp_z = 0.01;
% k_swfv_z = 10;
% 
% k_tp = 10;
% k_tv = 10;
% 
% k = 1000;
% k2 = 100;
%% Force computation

k = 0; % (1) ok
k2 = 0; % (0.1) ok
k_tp = 100; % 100
k_tv = 100; % 100

kh = 10; % 10
ks = 0.01; % 100
alpha = 4*(1-ks)/pi;
tau = 1 / (ks + alpha*abs(q1));
f_h = [max(-kh*x_h, 0); max(-kh*(z_h - l1),0)];
f_swf = [tau*cos(q2); tau*sin(q2)];
f_t = [(k_tp*(z_tr - z_t) - k_tv*dz_t)*cos(q3); (k_tp*(z_tr - z_t) - k_tv*dz_t)*sin(q3)];

% f_h = [k_hp*(x_hd - x_h) + k_hv*(dx_hd - dx_h); 0];
% % f_swf = [k_swfp_x*(x_swfd - x_swf) - k_swfv_x*dx_swf; -k_swfp_z*q2 - k_swfv_z*dz_swf];
% f_swf = [k_swfp_x*(x_swfd - x_swf) - k_swfv_x*dx_swf; -50];
% f_t = [-k*q3 - k2*dq3; k_tp*(z_tr - z_t) - k_tv*dz_t];

%% Matrices evaluation
J = eval_J(q);
B = eval_B();
Bp = pinv(B);
F = [f_h; f_swf; f_t];

% Input computation
u = Bp*J'*F
u = max(min(u, 30), -30); 

% % extract parameters
% kp1 = parameters(1);
% kp2 = parameters(2);
% kd1 = parameters(3);
% kd2 = parameters(4);
% alpha = parameters(5);
% 
% y1 = q(3) - alpha;
% y2 = -q(2) - q(1);
% dy1 = dq(3);
% dy2 = -dq(2) - dq(1);
% 
% % controller for torso
% u1 = kp1*y1 + kd1*dy1;
% 
% % controller for legs
% u2 = kp2*y2 + kd2*dy2; 
% u = [u1; u2];
% u = max(min(u, 30), -30); 
% saturate the output torque
end


%%% Marche pied levé %%%
% k_hp = 0.5;
% k_hv = 0.5;
% 
% k_swfp_x = 2; % 100
% k_swfv_x = 1; % 1
% k_swfp_z = 2;
% k_swfv_z = 1;
% 
% k_tp = 100; % 1000
% k_tv = 100; % 1000
% k = 1; % (1) ok
% k2 = 0.1; % (0.1) ok
% f_h = [k_hp*(x_hd - x_h) + k_hv*(dx_hd - dx_h); 0];
% % f_swf = [k_swfp_x*(x_swfd - x_swf) - k_swfv_x*dx_swf; -k_swfp_z*q2 - k_swfv_z*dz_swf];
% f_swf = [k_swfp_x*(x_swfd - x_swf) - k_swfv_x*dx_swf; 100];
% f_t = [-k*q3 - k2*dq3; k_tp*(z_tr - z_t) - k_tv*dz_t];