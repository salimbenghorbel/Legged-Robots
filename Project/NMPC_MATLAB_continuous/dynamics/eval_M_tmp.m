function M = eval_M_tmp(l1,l2,l3,m1,m2,m3,q1,q2,q3)
%EVAL_M_TMP
%    M = EVAL_M_TMP(L1,L2,L3,M1,M2,M3,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    24-Nov-2020 13:40:02

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = m1+m2+m3;
t9 = (l1.*m1.*t2)./2.0;
t10 = (l2.*m2.*t3)./2.0;
t11 = (l3.*m3.*t4)./2.0;
t12 = (l1.*m1.*t5)./2.0;
t13 = (l2.*m2.*t6)./2.0;
t14 = (l3.*m3.*t7)./2.0;
t15 = -t11;
t16 = -t12;
t17 = -t13;
M = reshape([t8,0.0,t9,t10,t15,0.0,t8,t16,t17,t14,t9,t16,(l1.^2.*m1)./4.0,0.0,0.0,t10,t17,0.0,(l2.^2.*m2)./4.0,0.0,t15,t14,0.0,0.0,(l3.^2.*m3)./4.0],[5,5]);
