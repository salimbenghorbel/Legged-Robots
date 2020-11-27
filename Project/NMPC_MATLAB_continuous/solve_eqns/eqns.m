function dy = eqns(t, y)
    % n this is the dimension of the ODE, note that n is 2*DOF, why? 
    % y1 = q1, y2 = q2, y3 = q3, y4 = dq1, y5 = dq2, y6 = dq3
    q = y(1:5);
    dq = y(6:end);

    % u = control(q, dq); % for the moment we set the control outputs to zero
    u = [0; 0];

    n = 10;
    dy = zeros(n, 1);

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
    dy(1:5) = dq;
    dy(6:end) = M \ (B*u + J'*F - C*dq - G);
end