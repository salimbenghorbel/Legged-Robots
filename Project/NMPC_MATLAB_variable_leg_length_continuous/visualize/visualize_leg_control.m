function visualize_leg_control(q, u3, u4)
    [m1, m2, m3, l1, l2, l3, ~] = set_parameters;
    length_ratio_reduction = 0.2;
    l1c = l1*(1 + length_ratio_reduction*(u3-1));
    l2c = l2*(1 + length_ratio_reduction*(u4-1));
    
    xg = q(1);
    zg = q(2);
    q1 = q(3);
    q2 = q(4);
    q3 = q(5);
    
    DeltaX = (m3*l3*sin(q3)-m1*l1*sin(q1)-m2*l2*sin(q2))/(m1+m2+m3);
    DeltaZ = (m3*l3*cos(q3)-m1*l1*cos(q1)-m2*l2*cos(q2))/(m1+m2+m3);
    
    xh = xg - DeltaX;
    zh = zg - DeltaZ;
    x1 = xh - l1*sin(q1)/2;
    z1 = zh - l1*cos(q1)/2;
    xl1 = xh - l1c*sin(q1);
    zl1 = zh - l1c*cos(q1);
    x2 = xh - l2*sin(q2)/2;
    z2 = zh - l2*cos(q2)/2;
    xl2 = xh - l2c*sin(q2);
    zl2 = zh - l2c*cos(q2);
    x3 = xh + l3*sin(q3)/2;
    z3 = zh + l3*cos(q3)/2;
    xt = xh + l3*sin(q3);
    zt = zh + l3*cos(q3);
    
    %% 
    % Here plot a schematic of the configuration of three link biped at the
    % generalized coordinate q = [q1, q2, q3]:
    lw = 2;
    % links
    plot([xl1, xh], [zl1, zh], 'linewidth', lw); 
    hold on
    plot([xl2, xh], [zl2, zh], 'linewidth', lw);
    plot([xh, xt], [zh, zt], 'linewidth', lw); 
    % plot a line for "ground"
    plot([-1, 1] + xg , [0, 0], 'color', 'black');
    axis 'square'
    xlim([-1, 1] + xg);
    ylim([-0.8, 1.2] + zg - 0.5);
    % point masses
    mz = 40;
    plot(x1, z1, '.', 'markersize', mz); 
    hold on
    plot(x2, z2, '.', 'markersize', mz); 
    plot(x3, z3, '.', 'markersize', mz);
end