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

%     fl1 = [0;min(k*exp(-alpha_k*zl1),1e4)] - (1/(1+min(exp(alpha_d*zl1),1e4))*[dxl1; dzl1]).*[dx;dz];
%     fl2 = [0;min(k*exp(-alpha_k*zl2),1e4)] - (1/(1+min(exp(alpha_d*zl2),1e4))*[dxl2; dzl2]).*[dx;dz];
%      fl1 = [0;k*(exp(-alpha_k*zl1))] - (1/(1+(exp(alpha_d*zl1)))*[dxl1; dzl1]).*[dx;dz];
%      fl2 = [0;k*(exp(-alpha_k*zl2))] - (1/(1+(exp(alpha_d*zl2)))*[dxl2; dzl2]).*[dx;dz];
fl1 = [5;5];
fl2 = [5;5];

end