function J = cost_function(X,U,e,data)

    J = 0;
    N = data.PredictionHorizon;
    gain_u = 2;
    gain_d = 100;
    gain_v = 5000;
    gain_posture = 5000;
    target_velocity = 0.6;
    % MATLAB uses row vectors instead of column vectors
    for i = 1:N-1
        %J = J + gain_u * norm(U(i,:))^2;
        J = J + gain_v * (X(i+1,6)- target_velocity)^2;
        %J = J + gain_posture * X(i+1,5)^2;
        J = J - gain_d*X(i+1,1)^2;
    end
    

end

