function dzl2 = eval_dzl2(q, dq, l2c)
    [m1, m2, m3, l1, l2, l3, ~] = set_parameters;
    q1 = q(3);
    q2 = q(4);
    q3 = q(5);
    dzg = dq(2);
    dq1 = dq(3);
    dq2 = dq(4);
    dq3 = dq(5);
    
    t2 = sin(q2);
    t3 = m1+m2+m3;
    t4 = 1.0./t3;
    dzl2 = dzg+dq2.*(l2c.*t2-l2.*m2.*t2.*t4)-dq1.*l1.*m1.*t4.*sin(q1)+dq3.*l3.*m3.*t4.*sin(q3);

%     t2 = sin(q2);
%     t3 = m1+m2+m3;
%     t4 = 1.0./t3;
%     dzl2 = dzg+dq2.*(l2.*t2-l2.*m2.*t2.*t4)-dq1.*l1.*m1.*t4.*sin(q1)+dq3.*l3.*m3.*t4.*sin(q3);
end