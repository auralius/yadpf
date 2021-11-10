function dps = trace_3X_1U(dps, x1_ic, x2_ic, x3_ic)

dps.u_star  = zeros(dps.n_horizon-1, 1);

nX1 = length(dps.X1);
nX2 = length(dps.X2);
nX3 = length(dps.X3);

% Initial stage is given by the IC
r = snap(x1_ic, min(dps.X1), max(dps.X1), nX1);
c = snap(x2_ic, min(dps.X2), max(dps.X2), nX2);
p = snap(x3_ic, min(dps.X3), max(dps.X3), nX3);
id = fastsub2ind3([nX1 nX2 nX3], r, c, p);

% Trace to the end horizon
fprintf('Forward tracing, please wait...\n')
for k = 1 : dps.n_horizon - 1 
    dps.u_star(k)  = dps.U(dps.U_star_matrix(k, id));        
    id = dps.descendant_matrix(k, id);
end

% Upsampling from Tocp to Tdyn
n = length(dps.u_star);
r = dps.T_ocp/dps.T_dyn;
dps.u_star  = conv(upsample(dps.u_star, r),ones(r,1));

% Trim and pad the last data, ZOH-method
dps.u_star = dps.u_star(1:n*r);
dps.u_star = [dps.u_star; repmat(dps.u_star(end),r,1)];

dps.x1_star = zeros(length(dps.u_star),1);
dps.x2_star = zeros(length(dps.u_star),1);
dps.x3_star = zeros(length(dps.u_star),1);

dps.x1_star(1) = x1_ic;
dps.x2_star(1) = x2_ic;
dps.x3_star(1) = x3_ic;

for k = 1 : length(dps.u_star)-1 % just ignore the last one
    [dps.x1_star(k+1), dps.x2_star(k+1), dps.x3_star(k+1)] = ...
        dps.state_update_fn(dps.x1_star(k), dps.x2_star(k), ...
        dps.x3_star(k), dps.u_star(k), dps.T_dyn);     
    
    dps.x1_star = min(max(dps.x1_star, dps.lb(1)), dps.ub(1));
    dps.x2_star = min(max(dps.x2_star, dps.lb(2)), dps.ub(2));
    dps.x3_star = min(max(dps.x3_star, dps.lb(3)), dps.ub(3));
end

fprintf('Complete!\n');

end