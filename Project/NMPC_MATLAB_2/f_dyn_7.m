function dx = f_dyn_7(x,u)
% n this is the dimension of the ODE
% x1 = q1, x2 = q2, x3 = q3, x4 = dq1, x5 = dq2, x6 = dq3
q = x(1:3);
dq = x(4:6);

M = eval_M(q);
C = eval_C(q, dq);
G = eval_G(q);
B = eval_B();
dx = [dq;M \ (B*u - C*dq - G);0];

end