function [u,opti] = eval_ctrl(x, opti, X0, X, U) 

% --- Set the initial state and reference --- 
opti.set_initial(X, rand(10,15+1)/100);
opti.set_value(X0, x);

% --- Setup solver NLP ---
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false); 
%ops.calc_lam_p = false;
opti.solver('ipopt', ops);

% --- Solve the optimization problem ---
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g)); 
end