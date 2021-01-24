tufunction objective_value = optimziation_fun(parameters, target_velocity, w, disturbance_list)

% extract parameters q0, dq0 and x
q0 = parameters(1:3);
dq0 = parameters(4:6);
x = parameters(7:end);

% run simulation
num_steps = 20; % the higher the better, but slow
sln = solve_eqns(q0, dq0, num_steps, x, disturbance_list);
[effort, distance, velocity, CoT, isFalling, stepLength, stepFrequency] = analyse(sln, x, false);

% calculate metrics such as distance, mean velocity and cost of transport
max_actuation = 30;
%target_velocity = 2;

w1 = w(1);
w2 = w(2);
w3 = w(3);
w4 = w(4);
w5 = w(5);

objective_value = w1*abs(target_velocity - velocity) + w2*CoT - w3*distance + w4*stepLength^2 + w5*stepFrequency^2;

% handle corner case when model walks backwards (e.g., objective_value =
% 1000)
if distance < 0
   objective_value = 10000;
end

% handle case when model falls (e.g., objective_value = 1000)
if isFalling
    objective_value = 10000;
end

end

