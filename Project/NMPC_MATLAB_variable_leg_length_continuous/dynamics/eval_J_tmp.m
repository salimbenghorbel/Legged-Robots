function J = eval_J_tmp(l1,l2,l3,l1c,l2c,m1,m2,m3,q1,q2,q3)
%EVAL_J_TMP
%    J = EVAL_J_TMP(L1,L2,L3,L1C,L2C,M1,M2,M3,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    29-Nov-2020 13:18:21

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = m1+m2;
t9 = m3+t8;
t10 = 1.0./t9;
t11 = l1.*m1.*t2.*t10;
t12 = l2.*m2.*t3.*t10;
t13 = l3.*m3.*t4.*t10;
t14 = l1.*m1.*t5.*t10;
t15 = l2.*m2.*t6.*t10;
t16 = l3.*m3.*t7.*t10;
t17 = -t13;
t18 = -t14;
t19 = -t15;
J = reshape([1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,t11,t18,t11-l1c.*t2,t18+l1c.*t5,t11,t18,t11,t18,t12,t19,t12,t19,t12-l2c.*t3,t19+l2c.*t6,t12,t19,t17,t16,t17,t16,t17,t16,l3.*t4.*t8.*t10,-l3.*t7.*t8.*t10],[8,5]);
