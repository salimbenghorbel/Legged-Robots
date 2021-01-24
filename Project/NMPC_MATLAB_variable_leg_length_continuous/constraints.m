function cineq = constraints(X,U,e,data)
% inequality constraints
[~, ~, ~, l1, l2, ~, ~] = set_parameters;
cineq = [];
N = data.PredictionHorizon;
for i =2:N+1
    q = X(i,1:5);
    length_ratio_reduction = 0.2;
    l1c = l1*(1 + length_ratio_reduction*(U(i,3)-1));
    l2c = l2*(1 + length_ratio_reduction*(U(i,4)-1));
    zl1 = eval_zl1(q, l1c);
    zl2 = eval_zl2(q, l2c);
    cineq = [cineq; -zl1; -zl2];
end

end