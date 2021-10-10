function dps = trace_3X_1U(dps, x1_ic, x2_ic, x3_ic)

x1_star = zeros(dps.n_horizon,  1);
x2_star = zeros(dps.n_horizon,  1);
x3_star = zeros(dps.n_horizon,  1);
u_star  = zeros(dps.n_horizon-1,1);

% Unpack the data, field access is slow
nX1               = length(dps.X1);
nX2               = length(dps.X2);
nX3               = length(dps.X3);
n_horizon         = dps.n_horizon;
X1                = dps.X1;
X2                = dps.X2;
X3                = dps.X3;
U_star_matrix     = dps.U_star_matrix;
descendant_matrix = dps.descendant_matrix;

% Initial stage is given by the IC
r = snap(x1_ic, min(X1), max(X1), nX1-1);
c = snap(x2_ic, min(X2), max(X2), nX2-1);
p = snap(x3_ic, min(X3), max(X3), nX3-1);
x1_star(1) = X1(r);
x2_star(2) = X2(c);
x3_star(3) = X3(p);

% Trace to the end horizon
fprintf('Froward tracing, please wait...\n')

id = sub2ind([nX1 nX2 nX3], r, c, p);
for k = 1 : n_horizon-1 
    u_star(k)  = U_star_matrix(id, k);    
    
    id = descendant_matrix(id,k);
    [x1_id,x2_id,x3_id] = ind2sub([nX1 nX2 nX3], id);               

    
    x1_star(k+1) = X1(x1_id);
    x2_star(k+1) = X2(x2_id);   
    x3_star(k+1) = X3(x3_id);   
end

fprintf('Complete!\n');

% Store the results
dps.x1_star = x1_star;
dps.x2_star = x2_star;
dps.x3_star = x3_star;
dps.u_star  = u_star;

end