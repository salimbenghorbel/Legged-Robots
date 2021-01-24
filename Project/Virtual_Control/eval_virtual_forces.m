function F = eval_virtual_forces(q, dq, parameters)
    [m1, m2, m3, l1, l2, l3, g] = set_parameters();
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    dq1 = dq(1);
    dq2 = dq(2);
    dq3 = dq(3);

    bh = parameters(1);
    bs = parameters(2);
    bt = parameters(3);
    q1_target = parameters(4);
    q2_target = parameters(5);
    q3_target = parameters(6);
    sigma_swf = parameters(7);
    f_lift = parameters(8);
    q1_trigger_lift = parameters(9);
    
    %% 
    q1_t = mod(q1+pi,2*pi)-pi;
    q2_t = mod(q2+pi,2*pi)-pi;
    q3_t = mod(q3+pi,2*pi)-pi;    
    
    %% Hip point
%     q1_target = pi/9;
%     bh = 100;
    kh = (2*bh^2 + g*l1*m1^2)/(2*l1*m1);
    Delta_q1 = q1_t - q1_target;
    Delta_q1_t = mod(Delta_q1+pi,2*pi)-pi;
    f_h = [(-kh*Delta_q1_t - bh*dq1)*cos(q1_t); (kh*Delta_q1_t + bh*dq1)*sin(q1_t)];
    f_h = (q1_t < 0).*f_h;

    %% Swing foot
%     sigma_swf = pi/18;
%     q2_target = -pi/9;
%     bs = 100;
    ks = (2*bs^2 - g*l2*m2^2)/(2*l2*m2);
    Delta_q2 = q2_t - q2_target;
    Delta_q2_t = mod(Delta_q2+pi,2*pi)-pi;
    f_swf = [(ks*Delta_q2_t + bs*dq2)*cos(q2_t); (-ks*Delta_q2_t - bs*dq2)*sin(q2_t)];
    f_swf = (1/(1+exp(-q1/(2*sigma_swf^2)))).*f_swf;
    
%     f_lift = 100;
%     q1_trigger_lift = -pi/12;
    f_swf = f_swf + (1/(1+exp((q1-q1_trigger_lift)/(2*sigma_swf^2)))).*[0; f_lift];

    %% Top point
%     q3_target = 0;
%     bt = 100;
    kt = (2*bt^2 + g*l3*m3^2)/(2*l3*m3);
    Delta_q3 = q3_t - q3_target;
    Delta_q3_t = mod(Delta_q3+pi,2*pi)-pi;
    f_t = [(-kt*Delta_q3_t - bt*dq3)*cos(q3_t); (kt*Delta_q3_t + bt*dq3)*sin(q3_t)];
    
    %% Force
    F = [f_h; f_swf; f_t];
end