% You can set any hyper parameters of the control function here; you may or
% may not want to use the step_number as the input of the function. 
function parameters = control_hyper_parameters(step_number)
    bh = 100;
    bs = 100;
    bt = 100;
    q1_target = pi/9;
    q2_target = -pi/9;
    q3_target = 0;
    sigma_swf = pi/18;
    f_lift = 100;
    q1_trigger_lift = -pi/12;
    
    parameters = [bh, bs, bt, q1_target, q2_target, q3_target, sigma_swf, f_lift, q1_trigger_lift]';
    parameters = [99.999397138911720;99.999535218056300;99.999302303901400;0.322662499736999;-0.267431047273216;0.004895439363258;0.145023635939721;99.999761979363240;-0.218653266810658];
end
