function [fl1, fl2] = eval_ground_forces(q, dq)
    zl1 = eval_zl1(q);
    zl2 = eval_zl2(q);
    dxl1 = eval_dxl1(q, dq);
    dzl1 = eval_dzl1(q, dq);
    dxl2 = eval_dxl2(q, dq);
    dzl2 = eval_dzl2(q, dq);
    
%     alpha_k = 50;
%     alpha_d = 1000;
%     k = 1000;
%     dx = 10;
%     dz = 1;
    
%     alpha_k = 50;
%     alpha_d = 50;
%     k = 1000;
%     dx = 10000;
%     dz = 1;

    alpha_k = 100;
    alpha_d = 100000;
    k = 10000;
    dx = 10000;
    dz = 1000;

%     fl1 = [0;min([k*exp(-alpha_k*zl1),1e8])] - (1/(1+exp(alpha_d*zl1))*[dxl1; dzl1]).*[dx;dz];
%     fl2 = [0;min([k*exp(-alpha_k*zl2),1e8])] - (1/(1+exp(alpha_d*zl2))*[dxl2; dzl2]).*[dx;dz];
[m1, m2, m3, l1, l2, l3, g] = set_parameters;
fg = (m1+m2+m3)*g;
fg = 500;
fl1 = [0; -dz*dzl1(zl1<=0)*fg] + [-dx*dxl1*(zl1<=0);0];
fl2 = [0; -dz*dzl1(zl2<=0)*fg] + [-dx*dxl2*(zl2<=0);0];
end