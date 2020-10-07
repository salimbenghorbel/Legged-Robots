function A_p = eval_A_p(q_p)
    [~, m, m3, l1, l2, l3, ~] = set_parameters;
    q1_p = q_p(1);
    q2_p = q_p(2);
    q3_p = q_p(3);

    t2 = l1.^2;
    t3 = l2.^2;
    t4 = l3.^2;
    t5 = -q2_p;
    t6 = -q3_p;
    t7 = q1_p+t5;
    t8 = q1_p+t6;
    t11 = (m.*t3)./4.0;
    t12 = (m3.*t4)./4.0;
    t9 = cos(t7);
    t10 = cos(t8);
    t13 = -t11;
    t14 = -t12;
    t15 = (l1.*l2.*m.*t9)./2.0;
    t16 = (l1.*l3.*m3.*t10)./2.0;
    t17 = -t16;
    A_p = reshape([t15+t17-m.*t2.*(5.0./4.0)-m3.*t2,t15,t17,t13+t15,t13,0.0,t14+t17,0.0,t14],[3,3]);
end