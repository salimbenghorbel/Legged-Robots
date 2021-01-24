function dx = f_dyn(x, uc)
    [~, ~, ~, l1, l2, ~, ~] = set_parameters;
    
    q = x(1:5);
    dq = x(6:end);
    
%     dx = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];

    u = uc(1:2);
    
    length_ratio_reduction = 0.2;
    l1c = l1*(1 + length_ratio_reduction*(uc(3)-1));
    l2c = l2*(1 + length_ratio_reduction*(uc(4)-1));
    
    M = eval_M(q);
    C = eval_C(q, dq);
    G = eval_G(q);
    B = eval_B();
    J = eval_J(q, l1c, l2c);
    
    fh = [0;0];
    [fl1, fl2] = eval_ground_forces(q, dq, l1c, l2c);
    ft = [0;0];
    F = [fh;fl1;fl2;ft];
    
    % write down the equations for dy:
%     dx(1:5) = dq;
%     dx(6:end) = M \ (B*u + J'*F - C*dq - G);
    dx = [dq; M \ (B*u + J'*F - C*dq - G)];
end