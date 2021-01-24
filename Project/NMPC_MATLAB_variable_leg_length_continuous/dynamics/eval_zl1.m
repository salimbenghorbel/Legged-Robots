function zl1 = eval_zl1(q, l1c)
    [m1, m2, m3, l1, l2, l3, ~] = set_parameters;
    zg = q(2);
    q1 = q(3);
    q2 = q(4);
    q3 = q(5);
    
    t2 = cos(q1);
    zl1 = zg-l1c.*t2+(l2.*m2.*cos(q2)-l3.*m3.*cos(q3)+l1.*m1.*t2)./(m1+m2+m3);
    
%     t2 = cos(q1);
%     zl1 = zg-l1.*t2+(l2.*m2.*cos(q2)-l3.*m3.*cos(q3)+l1.*m1.*t2)./(m1+m2+m3);
end