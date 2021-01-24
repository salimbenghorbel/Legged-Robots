function u = control(t, q, dq, q0, dq0, step_number, parameters)
% You may call control_hyper_parameters and desired_outputs in this function
% you don't necessarily need to use all the inputs to this control function
[m1, m2, m3, l1, l2, l3, g] = set_parameters();
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

q1_t = mod(q1+pi,2*pi)-pi;
q2_t = mod(q2+pi,2*pi)-pi;
q3_t = mod(q3+pi,2*pi)-pi;

%% Hip point
q1_target = pi/9;
bh = 20;
kh = (2*bh^2 + g*l1*m1^2)/(2*l1*m1);
% kh = (2*bh^2 + g*l1^3*m1^2)/(2*l1^2*m1);
% qh_eq = (q1_target*(2*bh^2 + g*l1^3*m1^2))/(2*l1^2*m1*((g*l1*m1)/2 - (2*bh^2 + g*l1^3*m1^2)/(2*l1^2*m1)));
Delta_q1 = q1_t - q1_target;
Delta_q1_t = mod(Delta_q1+pi,2*pi)-pi;

% f_h = [(-kh*Delta_q1_t - bh*dq1)*cos(q1_t); (kh*abs(Delta_q1_t) + bh*dq3*sign(Delta_q1_t))*sin(abs(q1_t))];
f_h = [(-kh*Delta_q1_t - bh*dq1)*cos(q1_t); (kh*abs(Delta_q1_t) + bh*dq1*sign(Delta_q1_t))*sin(abs(q1_t))];
f_h = (Delta_q1_t < 0).*f_h;
% f_h = [max(-kh*x_h, 0); max(-kh*(z_h - l1),0)];



%% Swing foot
sigma = pi/12;
q2_target = -pi/9;
bs = 10.5;
% ks = (2*bs^2 - g*l2^3*m2^2)/(2*l2^2*m2);
ks = (2*bs^2 - g*l2*m2^2)/(2*l2*m2);
Delta_q2 = q2_t - q2_target;
Delta_q2_t = mod(Delta_q2+pi,2*pi)-pi;

f_swf = [-(ks*Delta_q2_t + bs*dq2)*cos(q2_t); -(-ks*abs(Delta_q2_t) - bs*dq2*sign(Delta_q2_t))*sin(abs(q2_t))];
% f_swf = (exp(-q1^2/(2*sigma^2))).*f_swf;
f_swf = (1/(1+exp(q1/(2*sigma^2)))).*f_swf;

% 
% kh = 50; % 10
% ks = 0.01; % 100
% alpha = 6*(1-ks)/pi;
% % tau = (1 / (ks + alpha*abs(q1)))*(q2 < -pi/12);
% tau = (1 / (ks + alpha*abs(q1)))*(q2 > -pi/12);
% f_swf = [tau*cos(q2); tau*sin(q2)];
% % f_t = [(k_tp*(z_tr - z_t) - k_tv*dz_t)*sign(pi/2-abs(q3))*cos(q3); (k_tp*(z_tr - z_t) - k_tv*dz_t)*sin(abs(q3))];



%% Top point
bt = 15;
kt = (2*bt^2 + g*l3*m3^2)/(2*l3*m3);
% kt = (2*bt^2 + g*l3^3*m3^2)/(2*l3^2*m3);

% k = (400*kd^2)/833 + 4107388413278159/140737488355328;
f_t = [(-kt*q3_t - bt*dq3)*cos(q3_t); (kt*abs(q3_t) + bt*dq3*sign(q3_t))*sin(abs(q3_t))];
% f_t = [k*abs(q3_t)*sign(pi/2-abs(q3_t))*cos(q3_t); k*abs(q3_t)*sin(abs(q3_t))];

% f_h = [k_hp*(x_hd - x_h) + k_hv*(dx_hd - dx_h); 0];
% % f_swf = [k_swfp_x*(x_swfd - x_swf) - k_swfv_x*dx_swf; -k_swfp_z*q2 - k_swfv_z*dz_swf];
% f_swf = [k_swfp_x*(x_swfd - x_swf) - k_swfv_x*dx_swf; -50];
% f_t = [-k*q3 - k2*dq3; k_tp*(z_tr - z_t) - k_tv*dz_t];




%% Matrices evaluation
J = eval_J(q);
B = eval_B();
Bp = pinv(B);
% F = [f_h; f_swf; f_t];
F = eval_virtual_forces(q, dq);

% Input computation
u = Bp*J'*F;
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



%%%%%% Nouveau %%%%%%%

% k = 50; % (1) ok
% k2 = 0; % (0.1) ok
% k_tp = 500; % 100
% k_tv = 10; % 100
% 
% kh = 1000; % 10
% ks = 0.01; % 100
% alpha = 6*(1-ks)/pi;
% tau = (1 / (ks + alpha*abs(q1)))*(q2 < -pi/12);
% f_h = [max(-kh*x_h, 0); max(-kh*(z_h - l1),0)];
% f_swf = [tau*cos(q2); tau*sin(q2)];
% % f_t = [(k_tp*(z_tr - z_t) - k_tv*dz_t)*sign(pi/2-abs(q3))*cos(q3); (k_tp*(z_tr - z_t) - k_tv*dz_t)*sin(abs(q3))];
% 
% q3_t = mod(q3+pi,2*pi)-pi;
% f_t = [k*abs(q3_t)*sign(pi/2-abs(q3_t))*cos(q3_t); k*abs(q3_t)*sin(abs(q3_t))];