function dpf = yadpf_trace(dpf, x0)

u_star(1:dpf.n_inputs) = deal({zeros(dpf.n_horizon-1, 1)});

r = cell(1, dpf.n_states);
for i = 1 : dpf.n_states
    r{i} = snap(x0(i), dpf.lb(i), dpf.ub(i), dpf.nX(i));
end

if dpf.n_states > 1
    id = sub2ind(dpf.nX, r{:});
else
    id = [r{:}];
end

% Trace to the end horizon
fprintf('Forward tracing, please wait...\n')

for k = 1 : dpf.n_horizon-1
    for i = 1 : dpf.n_inputs
        u_star{i}(k) = dpf.inputs{i}(dpf.U_star_matrix{i}(k, id));        
    end
    
    id = dpf.descendant_matrix(k, id);
end

% Upsampling from T_ocp to T_dyn
n = length(u_star{1});
r = dpf.T_ocp/dpf.T_dyn;

for i = 1 : dpf.n_inputs
    u_star{i}  = conv(upsample(u_star{i}, r),ones(r,1));

    % Trim and pad the last data, ZOH-method
    u_star{i} = u_star{i}(1:n*r);
    u_star{i} = [u_star{i}; repmat(u_star{i}(end),r,1)];
end
x_star = cell(1, dpf.n_states);
for i = 1 : dpf.n_states
    x_star{i} = zeros(length(u_star{1}),1);
    x_star{i}(1) = x0(i);
end

for k = 1 : length(u_star{1})-1 % just ignore the last one
    X = cell(1, dpf.n_states);
    for i = 1 : dpf.n_states
        X{i} = x_star{i}(k);
    end

    U = cell(1, dpf.n_inputs);
    for i = 1 : dpf.n_inputs
        U{i} = u_star{i}(k);
    end

    X = dpf.state_update_fn(X, U, dpf.T_dyn);     

    % Apply the constraints and store the value
    for i = 1 : dpf.n_states
        X{i} = min(max(X{i}, dpf.lb(i)), dpf.ub(i)); 
        x_star{i}(k+1) = X{i};
    end
end

fprintf('Complete!\n')

dpf.u_star = u_star;
dpf.x_star = x_star;

end