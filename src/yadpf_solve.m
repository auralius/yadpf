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

% Store the optimal next-state
descendant_matrix = zeros(dpf.n_horizon, nXX, 'uint32');

% For storing the optimal input
U_star_matrix(1:n_inputs) = deal({zeros(dpf.n_horizon,nXX, 'uint32')});

fprintf('Horizons : %i stages\n',dpf.n_horizon);
fprintf('State    : %i nodes\n', nXX);
fprintf('Input    : %i nodes\n', nUU);
fprintf('Calculating terminal cost...\n')

% X : all possible state combination (nodes)
a = cell(1, n_states);
[a{:}] = ind2sub(nX, 1:nXX);

X = cell(1,  n_states);
for i = 1 : n_states
    X{i} = dpf.states{i}(a{i});
end

% Terminal cost
% J is a row vector with nXX elements, where nXX is the total number of
% all possible discretized state combination. At the end of ot, we add a
% very large number as a cost for the infeasible states.
J = dpf.terminal_cost_fn(X);
J = [J LARGE_NUMBER];
clear X;

% Create X and U, their dimensiona are: nUU times nXX
i = fastreprowvec(1:nXX, nUU);
a = cell(1, n_states);
[a{:}]  = ind2sub(nX, i);

i = fastrepcolvec((1:nUU)', nXX);
b = cell(1, n_inputs);
[b{:}]  = ind2sub(nU, i);

X = cell(1,  n_states);
for i = 1 : n_states
    X{i} = dpf.states{i}(a{i});
end

U = cell(1,  n_inputs);
for i = 1 : n_inputs
    U{i} = dpf.inputs{i}(b{i});
end

% Simulate the system with sampling period of Tdyn
fprintf('Simulating the system one step ahead...\n');
fprintf('Progress: ')

n_iter = dpf.T_ocp / dpf.T_dyn; 

ll = 0;
X_next = X;
for i = 1 : 1 : n_iter
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%', i/n_iter*100);
    X_next = dpf.state_update_fn(X_next, U, dpf.T_dyn);
end

% Bound the states within predefined constraints
r = cell(1, n_states);
infeasible = cell(1, n_states);
for i = 1 : n_states   
    [r{i}, infeasible{i}] = snap(X_next{i}, fastsca2mat(lb(i),nUU,nXX), ...
                  fastsca2mat(ub(i),nUU,nXX), fastsca2mat(nX(i),nUU,nXX));    
end

if n_states > 1
    next_ind = sub2ind(nX,r{:});
else
    next_ind = [r{:}];
end

% Find next states that are outside the boudaries, direct the index toward
% J(nXX+1)
next_index_ = next_ind;
for i = 1:n_states
    next_index_(ind2sub(nX,infeasible{i})) = nXX + 1;
end

clear X_next r infeasible;

fprintf('\nComplete!\n');

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = dpf.n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);
    
    J_old = J;
          
    [J_min, J_min_idx] = min(dpf.stage_cost_fn(X, U, k, dpf.T_ocp) + ...
                             reshape(J_old(next_index_), nUU, nXX), [], 1);
      
    descendant_matrix(k,:) = next_ind(fastsub2ind2([nUU nXX], ...
                                      J_min_idx, 1:nXX))';
    
    [b{:}]  = ind2sub(nU, J_min_idx);
    for i = 1 : n_inputs
        U_star_matrix{i}(k,:) = b{i};    
    end

    J(1:nXX) = J_min;    
end

fprintf('\nComplete!\n')

% Store the results
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

end
%------------- END OF CODE --------------
