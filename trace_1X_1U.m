function dps = trace_1X_1U(dps, x_ic)

x_star = zeros(dps.n_horizon,   1);
u_star = zeros(dps.n_horizon-1, 1);

% Unpack the data, field access is slow
nX               = length(dps.X);
n_horizon        = dps.n_horizon;
X                = dps.X;
U_star_matrix    = dps.U_star_matrix;
decendent_matrix = dps.decendent_matrix;

id = snap(x_ic, min(X), max(X), nX-1);
for k = 1 : n_horizon             
    x_star(k) = X(id);
    
    if (k < n_horizon)
        u_star(k) = U_star_matrix(id, k);    
    end
    
    id = decendent_matrix(id,k);
end


% Store the results
dps.x_star = x_star;
dps.u_star = remove_spike(u_star);
end