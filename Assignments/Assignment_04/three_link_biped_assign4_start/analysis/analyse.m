function [effort, distance, velocity, CoT, isFalling] = analyse(sln, parameters, to_plot)

kp1 = parameters(1);
kp2 = parameters(2);
kd1 = parameters(3);
kd2 = parameters(4);
alpha = parameters(5);
Umax = 30;

t = [];
xh = [];
dxh = [];
dxh_mean = [];
x_total = [];
zh = [];
dzh = [];
q1 = [];
q2 = [];
q3 = [];
dq1 = [];
dq2 = [];
dq3 = [];
u1 = [];
u2 = [];
step_length = [];
step_frequency = [];

% calculate gait quality metrics (distance, step frequency, step length,
% velocity, etc.)

for i = 1:length(sln.Y)
    t = [t; sln.T{i}];
    q1_i = sln.Y{i}(:,1);
    q2_i = sln.Y{i}(:,2);
    q3_i = sln.Y{i}(:,3);
    dq1_i = sln.Y{i}(:,4);
    dq2_i = sln.Y{i}(:,5);
    dq3_i = sln.Y{i}(:,6);
    q1 = [q1; q1_i];
    q2 = [q2; q2_i];
    q3 = [q3; q3_i];
    dq1 = [dq1; dq1_i];
    dq2 = [dq2; dq2_i];
    dq3 = [dq3; dq3_i];
    xh_i = [];
    dxh_i = [];
    zh_i = [];
    dzh_i = [];
    
    size_Y = size(sln.Y{i});
    for j = 1:size_Y(1)
%         sln.Y{i}(j,:)
        [x_h_j, z_h_j, dx_h_j, dz_h_j] = kin_hip(sln.Y{i}(j,1:3)', sln.Y{i}(j,4:end)');
        xh_i = [xh_i; x_h_j];   
        dxh_i = [dxh_i; dx_h_j];
        zh_i = [zh_i; z_h_j];
        dzh_i = [dzh_i; dz_h_j];
        
        y1_j = q3_i(j) - alpha;
        y2_j = -q2_i(j) - q1_i(j);
        dy1_j = dq3_i(j);
        dy2_j = -dq2_i(j) - dq1_i(j);
        % calculate actuation (you can use the control function)
        u1 = [u1; max(min(kp1*y1_j + kd1*dy1_j, 30), -30)];
        u2 = [u2; max(min(kp2*y2_j + kd2*dy2_j, 30), -30)];
    end
    xh = [xh; xh_i];
    dxh = [dxh; dxh_i];
    dxh_mean = [dxh_mean; mean(dxh_i)];
    zh = [zh; zh_i];
    if length(x_total) == 0
        x_total = [xh_i - xh_i(1)];
    else
        x_total = [x_total; x_total(end) + xh_i - xh_i(1)];
    end
    
    step_length = [step_length; xh_i(end) - xh_i(1)];
    step_frequency = [step_frequency; 1/(sln.T{i}(end) - sln.T{i}(1))];
end

effort = 0;
for i = 1:length(u1)
    effort = effort + (u1(i)^2 + u2(i)^2)/(2*t(end)*Umax);
end
distance = x_total(end) - x_total(1);
velocity = dxh_mean(end);
CoT = effort/distance;
if (min(zh) <= 0)
    isFalling = true;
else
    isFalling = false;
end

if to_plot    
    disp("final time = " + t(end));
    disp("effort = " + effort);
    disp("CoT    = " + CoT);
    disp("min velocity : " + min(dxh));
    disp("max velocity : " + max(dxh));
    disp("final mean velocity : " + velocity);
    disp("distance : " + distance);

    figure
    
    % plot the angles
    subplot 331
    hold on;
    plot(t, q1)
    plot(t, q2)
    plot(t, q3)
    legend('q1', 'q2', 'q3');
    title("Joint angles");
    
    % plot the hip position
    subplot 332
    hold on;
    plot(t, xh);
    plot(t, zh);
    plot(t, x_total);
    legend("xh", "zh", "x_total");
    title("hip position");
    
    % plot instantaneous and average velocity
    subplot 333
    %plot(t, dxh);
    plot(dxh_mean);
    title("average velocity");
    
    % plot projections of the limit cycle
    subplot 334
    plot(q1, dq1);
    title("dq1 / q1");
    subplot 335
    plot(q2, dq2);
    title("dq2 / q2");
    subplot 336
    plot(q3, dq3);
    title("dq3 / q3");
    
    % plot actuation
    subplot 337
    hold on;
    plot(t, u1);
    plot(t, u2);
    legend("u1", "u2");
    
    subplot 338
    hold on
    plot(step_length);
    title("step length vs step number");
    
    subplot 339
    hold on
    plot(2:1:length(sln.Y), step_frequency(2:end));
    title("frenquency step vs step number");
end

results = [effort, CoT];
end