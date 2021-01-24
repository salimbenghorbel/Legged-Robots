function dy = eqns(t, y, l1c, l2c, parameters)
    [~, ~, ~, l1, l2, ~, ~] = set_parameters;
    % n this is the dimension of the ODE, note that n is 2*DOF, why? 
    % y1 = q1, y2 = q2, y3 = q3, y4 = dq1, y5 = dq2, y6 = dq3
    q = y(1:5);
    dq = y(6:end);

    % u = control(q, dq); % for the moment we set the control outputs to zero
%     u = control(t, q, dq, parameters); 
    u = [0; 0];
       
    length_ratio_reduction = 0.2;
    l1c = l1*(1 + length_ratio_reduction*(l1c-1));
    l2c = l2*(1 + length_ratio_reduction*(l2c-1));

    n = 10;
    dy = zeros(n, 1);

    M = eval_M(q);
    C = eval_C(q, dq);
    G = eval_G(q);
    B = eval_B();
    J = eval_J(q,l1c,l2c);
    
    fh = [0;0];
    [fl1, fl2] = eval_ground_forces(q, dq, l1c, l2c);
    ft = [0;0];
    F = [fh;fl1;fl2;ft];
    
    % write down the equations for dy:
    dy(1:5) = dq;
    dy(6:end) = M \ (B*u + J'*F - C*dq - G);
end