function dps = trace_1X_1U(dps, x_ic)

dps.u_star  = zeros(dps.n_horizon-1, 1);

id = snap(x_ic, min(dps.X), max(dps.X), dps.nX);

% Trace to the end horizon
fprintf('Forward tracing, please wait...\n')
for k = 1 : dps.n_horizon-1    
    dps.u_star(k) = dps.U(dps.U_star_matrix(k, id));            
    id = dps.descendant_matrix(k, id);        
end

dps.u_star = remove_spike(dps.u_star, 1);

% Upsampling from Tocp to Tdyn
n = length(dps.u_star);
r = dps.T_ocp/dps.T_dyn;
dps.u_star  = conv(upsample(dps.u_star, r),ones(r,1));

% Trim and pad the last data, ZOH-method
dps.u_star = dps.u_star(1:n*r);
dps.u_star = [dps.u_star; repmat(dps.u_star(end),r,1)];

dps.x_star = zeros(length(dps.u_star),1);

dps.x_star(1) = x_ic;

for k = 1 : length(dps.u_star)-1 % just ignore the last one
    dps.x_star(k+1) = ...
        dps.state_update_fn(dps.x_star(k), ...
                            dps.u_star(k), dps.T_dyn);     
    
    dps.x_star = min(max(dps.x_star, dps.lb(1)), dps.ub(1));    
end

fprintf('Complete!\n');

end