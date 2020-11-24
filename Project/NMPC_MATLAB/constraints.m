function cineq = constraints(X,U,e,data)
% inequality constraints

cineq = [];
N = data.PredictionHorizon;
for i =2:N+1
    [~, z_h,~ , ~] = kin_hip(X(i,1:3),X(i,4:end));
    cineq = [cineq; -z_h];
end



end

