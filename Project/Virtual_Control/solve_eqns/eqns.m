function dy = eqns(t, y, t0, t10, y0, step_number, parameters, disturbance_list, h)
% n this is the dimension of the ODE, note that n is 2*DOF, why? 
% y1 = q1, y2 = q2, y3 = q3, y4 = dq1, y5 = dq2, y6 = dq3
% y0 is the states right after impact

q = [y(1); y(2); y(3)];
dq = [y(4); y(5); y(6)];

q0 = [y0(1); y0(2); y0(3)];
dq0 = [y0(4); y0(5); y0(6)];

M = eval_M(q);
C = eval_C(q, dq);
G = eval_G(q);
B = eval_B();

Jh = eval_Jh(q);
F_h_ext = [0; 0];
if step_number >= 10
    if t - t10 < 0.2
        F_h_ext = [0; 0];
    end
end

perturbation = disturbance_list(mod(10*step_number + floor((t-t0)/h),99)+1);
qd = q + [0; 0; perturbation];
dqd = dq + [0; 0; 0];
u = control(t, qd, dqd, q0, dq0, step_number, parameters); 

n = 6;   
dy = zeros(n, 1);
dy(1) = y(4);
dy(2) = y(5);
dy(3) = y(6);

dy(4:6) = M \ (-C*dq - G + B*u + Jh'*F_h_ext);

% u = [0;0];
% dy(4:6) = M \ (-C*dq - G + B*u + J'*[0;0;10;0;0;100]);

end