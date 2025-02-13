function x_next = RK4_f(x,u,h,f)
%
% Inputs :
%    x, u current state and input
%    h    sample period
%    f   continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%

% Runge-Kutta 4 integration
k1 = f(x,        u);
k2 = f(x+h/2*k1, u);
k3 = f(x+h/2*k2, u);
k4 = f(x+h*k3,   u);
x_next = x + h/6*(k1+2*k2+2*k3+k4);


q_next = x_next(1:3);
dq_next = x_next(4:6);
[x_swf, z_swf, ~, ~] = kin_swf(q_next, dq_next);
% if swf touches ground, perform impact map.
if  z_swf + 0.01 * cos(q_next(1)) + 0.0001 <=0
    [q_impact_next, dq_impact_next] = impact(q_next, dq_next);
    x_next = [q_impact_next; dq_impact_next; x_next(7) + x_swf];
end



%x_next = [q_next; dq_next].*(z_swf >= -0.001) + [q_impact_next; dq_impact_next].*(z_swf < -0.001);

%x_next = x_next.*(z_swf >= -0.001) + x.*(z_swf < -0.001);
%x_next = x_next.*(z_swf >= -0.001);
end