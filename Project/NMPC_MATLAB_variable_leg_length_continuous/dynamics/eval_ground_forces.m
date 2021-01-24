function [fl1, fl2] = eval_ground_forces(q, dq, l1c, l2c)
%     zl1 = eval_zl1(q);
%     zl2 = eval_zl2(q);
%     dxl1 = eval_dxl1(q, dq);
%     dzl1 = eval_dzl1(q, dq);
%     dxl2 = eval_dxl2(q, dq);
%     dzl2 = eval_dzl2(q, dq);
    
    zl1 = eval_zl1(q, l1c);
    zl2 = eval_zl2(q, l2c);
    dxl1 = eval_dxl1(q, dq, l1c);
    dzl1 = eval_dzl1(q, dq, l1c);
    dxl2 = eval_dxl2(q, dq, l2c);
    dzl2 = eval_dzl2(q, dq, l2c);
    
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

    %%%%%%%
    alpha_k = 200;
    alpha_d = 200;
    k = 1000;
    dx = 1000000; % 100000
    dz = 10000;
    %%%%%%%%%%%%
    
%     alpha_k = 100;
%     alpha_d = 100;
%     k = 1000;
%     dx = 100000;
%     dz = 10000;

    max_ground_force_x = 5000;
    max_ground_force_z = 2000;
    max_ground_force = [max_ground_force_x; max_ground_force_z];
%     fl1 = k*exp(-alpha_k*zl1)*[0;1] - (1/(1+exp(alpha_d*zl1))*[dxl1; dzl1]).*[dx;dz];
%     fl2 = k*exp(-alpha_k*zl2)*[0;1] - (1/(1+exp(alpha_d*zl2))*[dxl2; dzl2]).*[dx;dz];
%     
%     fl1 = max(min(k*exp(-alpha_k*zl1)*[0;1] - (1/(1+exp(alpha_d*zl1))*[dxl1; dzl1]).*[dx;dz] ,max_ground_force),-max_ground_force);
%     fl2 = max(min(k*exp(-alpha_k*zl2)*[0;1] - (1/(1+exp(alpha_d*zl2))*[dxl2; dzl2]).*[dx;dz] ,max_ground_force),-max_ground_force);
%       fl1 = [0; 1/(1+exp(alpha_k*zl1))*500] - (1/(1+exp(alpha_d*zl1))*[dxl1; dzl1]).*[dx;dz];
%       fl2 = [0; 1/(1+exp(alpha_k*zl2))*500] - (1/(1+exp(alpha_d*zl2))*[dxl2; dzl2]).*[dx;dz];
% fl1 = [0; (zl1<0)*500] - (1/(1+exp(alpha_d*zl1))*[dxl1; dzl1]).*[dx;dz];
% fl2 = [0; (zl2<0)*500] - (1/(1+exp(alpha_d*zl2))*[dxl2; dzl2]).*[dx;dz];
[m1, m2, m3, l1, l2, l3, g] = set_parameters;
fg = (m1+m2+m3)*g;
fg = 500;
fl1 = [0; -dz*dzl1(zl1<=0)*fg] + [-dx*dxl1*(zl1<=0);0];
fl2 = [0; -dz*dzl1(zl2<=0)*fg] + [-dx*dxl2*(zl2<=0);0];
%     alpha_s = -1;
%     sl1 = 1/(1+exp(alpha_s*dzl1));
%     sl2 = 1/(1+exp(alpha_s*dzl2));
%     fl1 = max(min(k*exp(-sl1*alpha_k*zl1)*[0;1] - (1/(1+exp(sl1*alpha_d*zl1))*[dxl1; dzl1]).*[dx;dz] ,max_ground_force),-max_ground_force);
%     fl2 = max(min(k*exp(-sl2*alpha_k*zl2)*[0;1] - (1/(1+exp(sl2*alpha_d*zl2))*[dxl2; dzl2]).*[dx;dz] ,max_ground_force),-max_ground_force);
end