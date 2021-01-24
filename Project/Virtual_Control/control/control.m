function u = control(t, q, dq, q0, dq0, step_number, parameters)
% You may call control_hyper_parameters and desired_outputs in this function
% you don't necessarily need to use all the inputs to this control function

%% Matrices evaluation
J = eval_J(q);
B = eval_B();
Bp = pinv(B);
F = eval_virtual_forces(q, dq, parameters);

% Input computation
u = Bp*J'*F;
u = max(min(u, 30), -30); 

end