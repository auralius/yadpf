function dps = trace_2X_2U(dps, x1_ic, x2_ic)

x1_star = zeros(dps.n_horizon,  1);
x2_star = zeros(dps.n_horizon,  1);
u1_star = zeros(dps.n_horizon-1,1);
u2_star = zeros(dps.n_horizon-1,1);

% Unpack the data, field access is slow
nX1              = length(dps.X1);
nX2              = length(dps.X2);
n_horizon        = dps.n_horizon;
X1               = dps.X1;
X2               = dps.X2;
U1_star_matrix    = dps.U1_star_matrix;
U2_star_matrix    = dps.U2_star_matrix;
decendent_matrix = dps.decendent_matrix;

% Initial stage is given by the IC
r = snap(x1_ic, min(X1), max(X1), nX1-1);
c = snap(x2_ic, min(X2), max(X2), nX2-1);

id = sub2ind([nX1 nX2], r, c);

% Trace to the end horizon
for k = 1 : n_horizon      
    [x1_id,x2_id] = ind2sub([nX1 nX2], id);
    
    % Convert index back to actual value
    x1 = X1(x1_id);
    x2 = X2(x2_id);
            
    x1_star(k) = x1;
    x2_star(k) = x2;
    
    if k < n_horizon
        u1_star(k)  = U1_star_matrix(id, k);
        u2_star(k)  = U2_star_matrix(id, k);
    end
    
    id = decendent_matrix(id,k);
end

% Store the results
dps.x1_star = x1_star;
dps.x2_star = x2_star;
dps.u1_star  = u1_star;
dps.u2_star  = u2_star;

end