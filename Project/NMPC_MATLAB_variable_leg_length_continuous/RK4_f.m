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

end