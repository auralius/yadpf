function dps = trace_3X_2U(dps, x1_ic, x2_ic, x3_ic)

dps.x1_star = zeros(dps.n_horizon,  1);
dps.x2_star = zeros(dps.n_horizon,  1);
dps.x3_star = zeros(dps.n_horizon,  1);
dps.u_star  = zeros(dps.n_horizon-1,1);

nX1 = length(dps.X1);
nX2 = length(dps.X2);
nX3 = length(dps.X3);

% Initial stage is given by the IC
r = snap(x1_ic, min(dps.X1), max(dps.X1), nX1);
c = snap(x2_ic, min(dps.X2), max(dps.X2), nX2);
p = snap(x3_ic, min(dps.X3), max(dps.X3), nX3);
dps.x1_star(1) = dps.X1(r);
dps.x2_star(2) = dps.X2(c);
dps.x3_star(3) = dps.X3(p);

% Trace to the end horizon
fprintf('Foward tracing, please wait...\n')

id = fastsub2ind3([nX1 nX2 nX3], r, c, p);
for k = 1 : dps.n_horizon-1 
    dps.u1_star(k)  = dps.U1(dps.U1_star_matrix(id, k));
    dps.u2_star(k)  = dps.U2(dps.U2_star_matrix(id, k));    
    
    id = dps.descendant_matrix(id,k);
    [x1_id,x2_id,x3_id] = ind2sub([nX1 nX2 nX3], id);               
    
    dps.x1_star(k+1) = dps.X1(x1_id);
    dps.x2_star(k+1) = dps.X2(x2_id);   
    dps.x3_star(k+1) = dps.X3(x3_id);   
end

fprintf('Complete!\n');

end