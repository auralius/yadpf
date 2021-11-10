function dps = trace_2X_2U(dps, x1_ic, x2_ic)

dps.u1_star = zeros(dps.n_horizon-1,1);
dps.u2_star = zeros(dps.n_horizon-1,1);

% Unpack the data, field access is slow
nX1 = length(dps.X1);
nX2 = length(dps.X2);

% Initial stage is given by the IC
r = snap(x1_ic, min(dps.X1), max(dps.X1), nX1);
c = snap(x2_ic, min(dps.X2), max(dps.X2), nX2);
id = fastsub2ind2([nX1 nX2], r, c);

% Trace to the end horizon
fprintf('Forward tracing, please wait...\n')

for k = 1 : dps.n_horizon-1
    dps.u1_star(k)  = dps.U1(dps.U1_star_matrix(k, id));
    dps.u2_star(k)  = dps.U2(dps.U2_star_matrix(k, id));    
    
    id = dps.descendant_matrix(k, id);
end

% Upsampling from T_ocp to T_dyn
n = length(dps.u1_star);
r = dps.T_ocp/dps.T_dyn;
dps.u1_star  = conv(upsample(dps.u1_star, r),ones(r,1));
dps.u2_star  = conv(upsample(dps.u2_star, r),ones(r,1));

% Trim and pad the last data, ZOH-method
dps.u1_star = dps.u1_star(1:n*r);
dps.u2_star = dps.u2_star(1:n*r);
dps.u1_star = [dps.u1_star; repmat(dps.u1_star(end),r,1)];
dps.u2_star = [dps.u2_star; repmat(dps.u2_star(end),r,1)];

dps.x1_star = zeros(length(dps.u1_star),1);
dps.x2_star = zeros(length(dps.u1_star),1);

dps.x1_star(1) = x1_ic;
dps.x2_star(1) = x2_ic;

for k = 1 : length(dps.u1_star)-1 % just ignore the last one
    [dps.x1_star(k+1), dps.x2_star(k+1)] = ...
        dps.state_update_fn(dps.x1_star(k), dps.x2_star(k), ...
        dps.u1_star(k), dps.u2_star(k), dps.T_dyn);     
    
    dps.x1_star = min(max(dps.x1_star, dps.lb(1)), dps.ub(1));
    dps.x2_star = min(max(dps.x2_star, dps.lb(2)), dps.ub(2));
end

fprintf('Complete!\n')

end