function F = eval_virtual_forces(q, dq)
    [m1, m2, m3, l1, l2, l3, g] = set_parameters();
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    dq1 = dq(1);
    dq2 = dq(2);
    dq3 = dq(3);

%     x_h = l1*sin(q1);
%     z_h = l1*cos(q1);
%     x_swf = l1*sin(q1) - l2*sin(q2);
%     z_swf = l1*cos(q1) - l2*cos(q2);
%     x_t = l1*sin(q1) + l3*sin(q3);
%     z_t = l1*cos(q1) + l3*cos(q3);
% 
%     dx_h = dq1*l1*cos(q1);
%     dz_h = -dq1*l1*sin(q1);
%     dx_swf = dq1*l1*cos(q1) - dq2*l2*cos(q2);
%     dz_swf = -dq1*l1*sin(q1) + dq2*l2*sin(q2);
%     dx_t = dq1*l1*cos(q1) + dq3*l3*cos(q3);
%     dz_t = -dq1*l1*sin(q1) - dq3*l3*sin(q3);
    
    %% 
    q1_t = mod(q1+pi,2*pi)-pi;
    q2_t = mod(q2+pi,2*pi)-pi;
    q3_t = mod(q3+pi,2*pi)-pi;

    
    
    %% Hip point
    q1_target = pi/9;
    bh = 100;
    kh = (2*bh^2 + g*l1*m1^2)/(2*l1*m1);
    Delta_q1 = q1_t - q1_target;
    Delta_q1_t = mod(Delta_q1+pi,2*pi)-pi;
%     f_h = [(-kh*Delta_q1_t - bh*dq1)*cos(q1_t); (kh*abs(Delta_q1_t) + bh*dq1*sign(Delta_q1_t))*sin(abs(q1_t))];
    f_h = [(-kh*Delta_q1_t - bh*dq1)*cos(q1_t); (kh*Delta_q1_t + bh*dq1)*sin(q1_t)];
%     f_h = (Delta_q1_t < 0).*f_h;
    f_h = (q1_t < 0).*f_h;



    %% Swing foot
    sigma = pi/18;
    q2_target = -pi/9;
    bs = 100; % 10.5
    ks = (2*bs^2 - g*l2*m2^2)/(2*l2*m2);
    Delta_q2 = q2_t - q2_target;
    Delta_q2_t = mod(Delta_q2+pi,2*pi)-pi;
%     f_swf = [-(ks*Delta_q2_t + bs*dq2)*cos(q2_t); (ks*abs(Delta_q2_t) + bs*dq2*sign(Delta_q2_t))*sin(abs(q2_t))];

%     f_swf = [-(ks*abs(Delta_q2_t) - bs*dq2*sign(Delta_q2_t))*sin(abs(q2_t)); (ks*Delta_q2_t - bs*dq2)*cos(q2_t)];

%     f_swf = [-(ks*Delta_q2_t - bs*dq2)*cos(q2_t); (ks*abs(Delta_q2_t) - bs*dq2*sign(Delta_q2_t))*sin(abs(q2_t))];
    f_swf = [(ks*Delta_q2_t + bs*dq2)*cos(q2_t); (-ks*Delta_q2_t - bs*dq2)*sin(q2_t)];
%     f_swf = [(ks*Delta_q2_t)*cos(q2_t); (-ks*Delta_q2_t)*sin(q2_t)];

%     f_swf = (exp(-q1^2/(2*sigma^2))).*f_swf;
    f_swf = (1/(1+exp(-q1/(2*sigma^2)))).*f_swf;


    %% Top point
    bt = 100;
    kt = (2*bt^2 + g*l3*m3^2)/(2*l3*m3);
%     f_t = [(-kt*q3_t - bt*dq3)*cos(q3_t); (kt*abs(q3_t) + bt*dq3*sign(q3_t))*sin(abs(q3_t))];
    f_t = [(-kt*q3_t - bt*dq3)*cos(q3_t); (kt*q3_t + bt*dq3)*sin(q3_t)];


    
    %% Force
    F = [f_h; f_swf; f_t];
end