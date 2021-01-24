%%
% This function animates the solution of the equations of motion of the
% three link biped. 
% sln is the solution computed by solve_eqns.m
%%
function animate(sln)

figure();
skip = 5; 

tic();
Y = sln.Y{1};%
[N, ~] = size(Y);
for i = 1:skip:N % what does skip do?
    q = Y(i,1:5)';%
    pause(0.002);  % pause for 2 mili-seconds
    % visualize :
    visualize(q);
    hold off
end
t_anim = toc();

% Real time factor is the actual duration of the simulations (get it from sln) to
% the time it takes for MATLAB to animate the simulations (get it from
% t_anim). How does 'skip' effect this value? what does a real time factor
% of 1 mean?

%% Answer :
% real_time_factor characterizes the scale time factor between reality and
% animation. Then, if the real_time_factor is equal to 1, the animation is
% exactly scaled to reality in terms of time duration. Finally, if the
% real_time_factor is greater than 1, the animation is faster than reality (smaller animation time duration).
% 'skip' enables us to skip animation frames, thus the animation is
% faster compared to the case where we consider all frames. Then, the real_time_factor increases if the 'skip' value increases.


real_time_factor = sln.T{end}(end) / t_anim;% This is only an estimation 
fprintf('Real time factor:');
disp(real_time_factor);
end