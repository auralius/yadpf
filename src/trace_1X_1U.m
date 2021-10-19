function dps = trace_1X_1U(dps, x_ic)

x_star = zeros(dps.n_horizon,   1);
u_star = zeros(dps.n_horizon-1, 1);

id = snap(x_ic, min(dps.X), max(dps.X), dps.nX);
x_star(1) = dps.X(id);

% Trace to the end horizon
fprintf('Forward tracing, please wait...\n')

for k = 1 : dps.n_horizon-1    
    u_star(k) = dps.U(dps.U_star_matrix(id, k));            
    id = dps.descendant_matrix(id,k);    
    x_star(k+1) = dps.X(id);    
end

fprintf('Complete!\n');

% Store the results
dps.x_star = x_star;
dps.u_star = remove_spike(u_star);
end