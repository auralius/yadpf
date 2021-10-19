function dps = trace_1X_2U(dps, x_ic)

dps.x_star = zeros(dps.n_horizon,  1);
dps.u1_star = zeros(dps.n_horizon-1,1);
dps.u2_star = zeros(dps.n_horizon-1,1);

nX = length(dps.X);

% Initial stage is given by the IC
id = snap(x_ic, min(dps.X), max(dps.X), nX);
dps.x_star(1) = dps.X(id);

% Trace to the end horizon
fprintf('Foward tracing, please wait...\n')

for k = 1 : dps.n_horizon-1
    dps.u1_star(k)  = dps.U1(dps.U1_star_matrix(id, k));
    dps.u2_star(k)  = dps.U2(dps.U2_star_matrix(id, k));    
    id = dps.descendant_matrix(id,k);        
    dps.x_star(k+1) = dps.X(id);    
end

fprintf('Complete!\n');

end