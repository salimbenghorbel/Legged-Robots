function J = eval_J(q)
    [~, ~, ~, l1, l2, l3, ~] = set_parameters();
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    
    t2 = cos(q1);
    t3 = sin(q1);
    t4 = l1.*t2;
    t5 = l1.*t3;
    t6 = -t5;
    J = reshape([t4,t6,t4,t6,t4,t6,0.0,0.0,-l2.*cos(q2),l2.*sin(q2),0.0,0.0,0.0,0.0,0.0,0.0,l3.*cos(q3),-l3.*sin(q3)],[6,3]);
end