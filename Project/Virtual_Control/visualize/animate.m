%%
% This function animates the solution of the equations of motion of the
% three link biped. 
% sln is the solution computed by solve_eqns.m
%%
function animate(sln, parameters)

figure();
skip = 10;
tic();
num_steps = length(sln.T);r0 = [0; 0];
for j = 1:num_steps
    Y = sln.Y{j};
    T = sln.T{j};
    [N, ~] = size(Y);
    for i = 1:skip:N
        q = Y(i, 1:3);
        dq = Y(i, 4:6);
        pause(0.02);
        visualize(q, dq, r0, parameters, j, T(i) - T(1));
        hold off
    end
    [x0, ~, ~, ~] = kin_swf(q);
    r0 = r0 + [x0; 0];
end
t_anim = toc();
real_time_factor = sln.T{end}(end) / t_anim;
fprintf('Real time factor:');
disp(real_time_factor);
end