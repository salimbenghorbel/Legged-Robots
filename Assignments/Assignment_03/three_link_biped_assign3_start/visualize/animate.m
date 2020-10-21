%%
% This function animates the solution of the equations of motion of the
% three link biped. 
% sln is the solution computed by solve_eqns.m
%%
function animate(sln)

figure();
skip = 5; 

num_steps = length(sln.Y);% total number of steps the robot has taken (find this from sln)
r0 = [0; 0];
tic();
for j = 1:num_steps
    Y = sln.Y{j};%
    [N, ~] = size(Y);
    for i = 1:skip:N % what does skip do?
        q = Y(i,1:3)';%
        pause(0.002);  % pause for 2 mili-seconds
        % visualize :
        visualize(q, r0);
        hold off
    end
    % update r0:
    dq = Y(i,4:end)';
    [x_swf, z_swf, ~, ~] = kin_swf(q, dq);
    r0 = r0 + [x_swf; z_swf];
    
end
t_anim = toc();

% Real time factor is the actual duration of the simulations (get it from sln) to
% the time it takes for MATLAB to animate the simulations (get it from
% t_anim). How does 'skip' effect this value? what does a real time factor
% of 1 mean?

% real_time_factor characterizes the scale time factor between reality and
% animation. Then, if the real_time_facotr is equal to 1, the animation is
% exactly scaled to reality in terms of time duration. Finally, if the
% real_time_factor is greater than 1, the animation is faster than reality.
% 'skip' enables us to skip animation frames, thus the animation is
% faster compared to the case where we consider all frames. Then, the real_time_factor increases if the 'skip' value increases.


real_time_factor = sln.T{end}(end) / t_anim;% This is only an estimation 
fprintf('Real time factor:');
disp(real_time_factor);
end