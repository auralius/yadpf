function dpf = yadpf_solve(dpf)
% Solve an optimal control problem with backward dynamic programming
%
% Syntax:  dpf = yadf_solve(dpf)
%
% Inputs:
%    dpf - The data structure for the optimal control problem
%
% Outputs:
%    dpf - The same data structure as the input, but with the results
%
% Other m-files required:
%   fastsca2mat.m
%   snap.m
%   fastreprowvec.m
%   fastrepcolvec.m
%   fastsub2ind2.m
%
% Author:
%   Auralius Manurung
%   Universitas Pertamina
%   auralius.manurung@ieee.org

%------------- BEGIN CODE --------------

% A vvery large number
LARGE_NUMBER = 1e10;

% Calculate number of the states, the inputs and the nodes
n_states = numel(dpf.states);
n_inputs = numel(dpf.inputs);

ub = cellfun(@max, dpf.states);
lb = cellfun(@min, dpf.states);
nX = cellfun('length', dpf.states);
nU = cellfun('length', dpf.inputs);

nXX = prod(nX);
nUU = prod(nU);

fprintf('Horizons : %i stages\n',dpf.n_horizon);
fprintf('State    : %i nodes\n', nXX);
fprintf('Input    : %i nodes\n', nUU);
fprintf('Calculating terminal cost...\n')

% Terminal cost
% J is a row vector with nXX elements, where nXX is the total number of
% all possible discretized state combination. At the end of ot, we add a
% very large number as a cost for the infeasible states.
X = generate_X(); % X contains all possible state combination (nodes)
J = dpf.terminal_cost_fn(X);
clear X; % We are done with X

% Create X and U, their dimensions are nUU times nXX
X = generate_X_for_all_U();
U = generate_U_for_all_X();

% Simulate the system with sampling period of Tdyn
fprintf('Simulating the system one step ahead...\n');
fprintf('Progress: ')

n_iter = dpf.T_ocp / dpf.T_dyn;

ll = 0;
for i = 1 : 1 : n_iter
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%', i/n_iter*100);
    X = dpf.state_update_fn(X, U, dpf.T_dyn);
end

% Bound the states within predefined constraints
fprintf('\nApplying the boundaries...\n');
r = cell(1, n_states);
infeas = cell(1, n_states);
for i = 1 : n_states
    [r{i}, infeas{i}] = snap(X{i}, lb(i), ub(i), nX(i));
end

next_ind = flexible_sub2ind(nX, r{:});

clear X r; % We are done with X and r

% Find next states that are outside the boudaries, preset a very high cost!
J_bounds = zeros(size(J));
for i = 1 : n_states
    J_bounds(next_ind(ind2sub(size(next_ind), infeas{i}))) = LARGE_NUMBER;
end

clear infeas; % We are done with infeasible

fprintf('Complete!\n');

% Reload the original X
X = generate_X_for_all_U();

% Store the optimal next-state
descendant_matrix = zeros(dpf.n_horizon, nXX, 'uint32');

% For storing the optimal input
U_star_matrix(1:n_inputs) = deal({zeros(dpf.n_horizon,nXX, 'uint32')});

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;
for k = dpf.n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);

    J_old = J;

    [J_min, J_min_idx] = min(dpf.stage_cost_fn(X, U, k, dpf.T_ocp) + ...
                             J_old(next_ind) + J_bounds(next_ind), [], 1);

    descendant_matrix(k,:) = next_ind(fastsub2ind2([nUU nXX], ...
                                      J_min_idx, 1:nXX))';

    b = flexible_ind2sub(nU, J_min_idx);
    for i = 1 : n_inputs
        U_star_matrix{i}(k,:) = b{i};
    end

    J(1:nXX) = J_min;
end

fprintf('\nComplete!\n')

clear U X next_ind J_bounds % Avoid possible insufficient memory

% Store the results
dpf.J = J;
dpf.descendant_matrix = descendant_matrix;
dpf.U_star_matrix     = U_star_matrix;

% Additional information that might be still needed
dpf.n_states = n_states;
dpf.n_inputs = n_inputs;
dpf.nX       = nX;
dpf.nU       = nU;
dpf.lb       = lb;
dpf.ub       = ub;
dpf.nUU      = nUU;
dpf.nXX      = nXX;

dpf.method   = 'dpa';

    function X = generate_X()
    % Generate vector X: 1 times nXX
        sub = flexible_ind2sub(nX, 1:nXX);
        X = cell(1, n_states);
        for n = 1 : n_states
            X{n} = dpf.states{n}(sub{n});
        end
        clear sub;
    end

    function X = generate_X_for_all_U()
    % Generate matrix X: nUU times nXX
        sub = flexible_ind2sub(nX, fastreprowvec(1:nXX, nUU));
        X = cell(1, n_states);
        for n = 1 : n_states
            X{n} = dpf.states{n}(sub{n});
        end
        clear sub;
    end

    function U = generate_U_for_all_X()
    % Generate matrix U: nUU times nXX
        sub = flexible_ind2sub(nU, fastrepcolvec((1:nUU)', nXX));
        U = cell(1,  n_inputs);
        for n = 1 : n_inputs
            U{n} = dpf.inputs{n}(sub{n});
        end
        clear sub;
    end

end
%------------- END OF CODE --------------
