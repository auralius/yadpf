function dps = trace_1X_2U(dps, x_ic)

dps.u1_star = zeros(dps.n_horizon-1,1);
dps.u2_star = zeros(dps.n_horizon-1,1);

nX = length(dps.X);

% Initial stage is given by the IC
id = snap(x_ic, min(dps.X), max(dps.X), nX);

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

dps.x_star = zeros(length(dps.u1_star),1);

dps.x_star(1) = x_ic;

for k = 1 : length(dps.u1_star)-1 % just ignore the last one
    dps.x_star(k+1) = ...
        dps.state_update_fn(dps.x_star(k), ...
        dps.u1_star(k), dps.u2_star(k), dps.T_dyn);     
    
    dps.x_star = min(max(dps.x_star, dps.lb(1)), dps.ub(1));    
end

fprintf('Complete!\n')

end