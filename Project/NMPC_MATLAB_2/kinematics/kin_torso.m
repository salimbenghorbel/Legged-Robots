function [x_t, z_t, dx_t, dz_t] = kin_torso(q,dq)
[~, ~, ~, l1, ~, l3, ~] = set_parameters();
q1 = q(1);
dq1 = dq(1);
q3 = q(3);
dq3 = dq(3);

x_t = l1*sin(q1) + l3*sin(q3);
z_t = l1*cos(q1) + l3*cos(q3);
dx_t = dq1*l1*cos(q1) + dq3*l3*sin(q3);
dz_t = -dq1*l1*sin(q1) - dq3*l3*sin(q3);

end

