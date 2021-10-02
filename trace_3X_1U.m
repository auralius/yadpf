function dps = trace_3X_1U(dps, x1_ic, x2_ic, x3_ic)

x1_star = zeros(dps.n_horizon,  1);
x2_star = zeros(dps.n_horizon,  1);
x3_star = zeros(dps.n_horizon,  1);
u_star  = zeros(dps.n_horizon-1,1);

% Unpack the data, field access is slow
nX1              = length(dps.X1);
nX2              = length(dps.X2);
nX3              = length(dps.X3);
n_horizon        = dps.n_horizon;
X1               = dps.X1;
X2               = dps.X2;
X3               = dps.X3;
U_star_matrix    = dps.U_star_matrix;
decendent_matrix = dps.decendent_matrix;

% Initial stage is given by the IC
r = snap(x1_ic, min(X1), max(X1), nX1-1);
c = snap(x2_ic, min(X2), max(X2), nX2-1);
p = snap(x3_ic, min(X3), max(X3), nX3-1);

id = sub2ind([nX1 nX2 nX3], r, c, p);

% Trace to the end horizon
for k = 1 : n_horizon      
    [x1_id, x2_id, x3_id] = ind2sub([nX1 nX2 nX3], id);
    
    % Convert index back to actual value
    x1 = X1(x1_id);
    x2 = X2(x2_id);
    x3 = X3(x3_id);
            
    x1_star(k) = x1;
    x2_star(k) = x2;
    x3_star(k) = x3;
    
    if k < n_horizon
        u_star(k)  = U_star_matrix(id, k);
    end
    
    id = decendent_matrix(id,k);
end

% Store the results
dps.x1_star = x1_star;
dps.x2_star = x2_star;
dps.x3_star = x3_star;
dps.u_star  = u_star;

end