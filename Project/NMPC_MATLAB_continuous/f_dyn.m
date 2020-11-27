function dx = f_dyn(x, u)

    q = x(1:5);
    dq = x(6:end);

    n = 10;
    dx = zeros(n, 1);

    M = eval_M(q);
    C = eval_C(q, dq);
    G = eval_G(q);
    B = eval_B();
    J = eval_J(q);
    
    fh = [0;0];
    [fl1, fl2] = eval_ground_forces(q, dq);
    ft = [0;0];
    F = [fh;fl1;fl2;ft];
    
    % write down the equations for dy:
    dx(1:5) = dq;
    dx(6:end) = M \ (B*u + J'*F - C*dq - G);
end