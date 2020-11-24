function J = cost_function(X,U,e,data)

    J = 0;
    N = data.PredictionHorizon;
    gain_u = 1;
    gain_d = 1000;
    gain_v = 1;
    
    % MATLAB uses row vectors instead of column vectors
    for i = 1:N-1
        %J = J + gain_u * norm(U(i,:))^2;
        [x_h, ~, dx_h, ~] = kin_hip(X(i+1,1:3),X(i+1,4:end));
        %J = J - gain_v * dx_h^2;
        J = J - gain_d * x_h^2;
    end
    

end

