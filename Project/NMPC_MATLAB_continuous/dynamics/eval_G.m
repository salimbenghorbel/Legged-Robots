function G = eval_G(q)
    [m1, m2, m3, l1, l2, l3, g] = set_parameters;
    q1 = q(3);
    q2 = q(4);
    q3 = q(5);
    
    G = [0.0;g.*(m1+m2+m3);g.*l1.*m1.*sin(q1).*(-1.0./2.0);g.*l2.*m2.*sin(q2).*(-1.0./2.0);(g.*l3.*m3.*sin(q3))./2.0];
end