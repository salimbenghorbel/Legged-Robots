function dxl1 = eval_dxl1(q, dq, l1c)
    [m1, m2, m3, l1, l2, l3, ~] = set_parameters;
    q1 = q(3);
    q2 = q(4);
    q3 = q(5);
    dxg = dq(1);
    dq1 = dq(3);
    dq2 = dq(4);
    dq3 = dq(5);
    
    t2 = cos(q1);
    t3 = m1+m2+m3;
    t4 = 1.0./t3;
    dxl1 = dxg-dq1.*(l1c.*t2-l1.*m1.*t2.*t4)+dq2.*l2.*m2.*t4.*cos(q2)-dq3.*l3.*m3.*t4.*cos(q3);

%     t2 = cos(q1);
%     t3 = m1+m2+m3;
%     t4 = 1.0./t3;
%     dxl1 = dxg-dq1.*(l1.*t2-l1.*m1.*t2.*t4)+dq2.*l2.*m2.*t4.*cos(q2)-dq3.*l3.*m3.*t4.*cos(q3);
end