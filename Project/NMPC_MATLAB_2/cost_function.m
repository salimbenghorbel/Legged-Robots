function J = cost_function(X,U,e,data)

    J = 0;
    N = data.PredictionHorizon;
    gain_u = 2;
    gain_d = 4;
    gain_v = 8;
    gain_posture = 1;
    
    % MATLAB uses row vectors instead of column vectors
    for i = 1:N-1
        J = J + gain_u * norm(U(i,:))^2;
        [x_h, ~, dx_h, ~] = kin_hip(X(i+1,1:3),X(i+1,4:6));
        J = J - gain_v * dx_h^2;
        %J = J - gain_d * x_h^2;
        J = J + gain_posture * X(i+1,3)^2;
        J = J - gain_d*X(i+1,7)^2;
    end
    

end

