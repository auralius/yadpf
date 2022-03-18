function dpf = yadpf_trace(dpf, x0)
% Forward tracing to extract the optimal inputs for a given initial condition
% By using the extracted optimal input, the sytem is simulated to calculate 
% the resulting optimal states
%
% Syntax:  yadpf_trace(dpf, x0)
%
% Inputs:
%    dpf - The data structure for the optimal control problem
%    x0  - The initial states (1xn vector)
%
% Outputs: -   
%    dpf - The data same data structure with the optimal inputs stored in
%          dpf.u_star and the optimal states in dpf.x_star
%
% Author:
%   Auralius Manurung
%   Universitas Pertamina 
%   auralius.manurung@ieee.org

%------------- BEGIN CODE --------------
%u_star(1:dpf.n_inputs) = deal({zeros(dpf.n_horizon-1, 1)});

u_star_lores(1:dpf.n_inputs) = deal({0});  % the original optimal inputs, before up-sampling
u_star(1:dpf.n_inputs)             = deal({0});              % the up-sampled optimal inputs 
x_star_lores(1:dpf.n_states) = deal({0});  % the optimal unsimulated states, this is taken from the descendant matrix

s_sub = cell(1, dpf.n_states);
for i = 1 : dpf.n_states
    s_sub{i} = snap(x0(i), dpf.lb(i), dpf.ub(i), dpf.nX(i));
    x_star_lores{i} = dpf.states{i}(s_sub{i});
end

if dpf.n_states > 1
    s_id = sub2ind(dpf.nX, s_sub{:});
else
    s_id = [s_sub{:}];
end

% Trace to the end horizon
fprintf('Forward tracing, please wait...\n')

for k = 1 : dpf.n_horizon-1
    for i = 1 : dpf.n_inputs
        u_star_lores{i}(k,1) = dpf.inputs{i}(dpf.U_star_matrix{i}(k, s_id));        
    end
    
    s_id = dpf.descendant_matrix(k, s_id);
    
    [s_sub{:}]  = ind2sub(dpf.nX, s_id); 
    
    for i = 1 : dpf.n_states
        x_star_lores{i}(k+1) = dpf.states{i}(s_sub{i});        
    end
end

% Upsampling from T_ocp to T_dyn
n = length(u_star_lores{1});
r = dpf.T_ocp/dpf.T_dyn;

for i = 1 : dpf.n_inputs
    u_star{i}  = conv(upsample(u_star_lores{i}, r),ones(r,1));

    % Trim and pad the last data, ZOH-method
    u_star{i} = u_star{i}(1:n*r);
    %u_star{i} = [u_star{i}; repmat(u_star{i}(end),r,1)];
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

dpf.u_star_unsimulated = u_star_lores; % before the upsampleg, taken from the U_star_matrix
dpf.u_star             = u_star;       % upsampled, used for the dynamic simulation

dpf.x_star_unsimulated = x_star_lores; % coarse, unsimulated, taken from the descendant matrix
dpf.x_star             = x_star;       % taken from the simulation results

end
%------------- END OF CODE --------------
