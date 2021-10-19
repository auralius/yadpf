function dps = trace_2X_2U(dps, x1_ic, x2_ic)

dps.x1_star = zeros(dps.n_horizon,  1);
dps.x2_star = zeros(dps.n_horizon,  1);
dps.u1_star = zeros(dps.n_horizon-1,1);
dps.u2_star = zeros(dps.n_horizon-1,1);

% Unpack the data, field access is slow
nX1 = length(dps.X1);
nX2 = length(dps.X2);

% Initial stage is given by the IC
r = snap(x1_ic, min(dps.X1), max(dps.X1), nX1);
c = snap(x2_ic, min(dps.X2), max(dps.X2), nX2);
dps.x1_star(1) = dps.X1(r);
dps.x2_star(1) = dps.X2(c);

% Trace to the end horizon
fprintf('Froward tracing, please wait...\n')

id = fastsub2ind2([nX1 nX2], r, c);
for k = 1 : dps.n_horizon-1
    dps.u1_star(k)  = dps.U1(dps.U1_star_matrix(id, k));
    dps.u2_star(k)  = dps.U2(dps.U2_star_matrix(id, k));    
    
    id = dps.descendant_matrix(id,k);
    [x1_id,x2_id] = ind2sub([nX1 nX2], id);   
    
    dps.x1_star(k+1) = dps.X1(x1_id);
    dps.x2_star(k+1) = dps.X2(x2_id); 
end

fprintf('Complete!\n');

end