function J = eval_J_tmp(l1,l2,l3,q1,q2,q3)
%EVAL_J_TMP
%    J = EVAL_J_TMP(L1,L2,L3,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    08-Dec-2020 18:02:58

t2 = cos(q1);
t3 = sin(q1);
t4 = l1.*t2;
t5 = l1.*t3;
t6 = -t5;
J = reshape([t4,t6,t4,t6,t4,t6,0.0,0.0,-l2.*cos(q2),l2.*sin(q2),0.0,0.0,0.0,0.0,0.0,0.0,l3.*cos(q3),-l3.*sin(q3)],[6,3]);
