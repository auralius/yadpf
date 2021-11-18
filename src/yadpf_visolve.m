function dpf = yadpf_visolve(dpf, tol)
% Solve an optimal control problem with value iteration
%
% Syntax:  dpf = yadf_solve(dpf)
%
% Inputs:
%    dpf - The data structure for the optimal control problem
%    tol - A small number that is used to terminate the value iteration 
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

if nargin < 2
    tol = 1e-3;
end

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
descendant_matrix = zeros(nXX,'uint32');

% For storing the optimal input
U_star_matrix(1:n_inputs) = deal({zeros(nXX, 'uint32')});

fprintf('State    : %i nodes\n', nXX);
fprintf('Input    : %i nodes\n', nUU);
fprintf('Calculating terminal cost...\n')

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
for i = 1 : n_states
    X_next{i} = min(max(X_next{i}, lb(i)), ub(i));
    r{i} = snap(X_next{i}, fastsca2mat(lb(i),nUU,nXX), ...
           fastsca2mat(ub(i),nUU,nXX), fastsca2mat(nX(i),nUU,nXX));    
end

if n_states > 1
    next_ind = sub2ind(nX,r{:});
else
    next_ind = [r{:}];
end

clear X_next r;

fprintf('\nComplete!\n');

% Stage-wise iteration
fprintf('Running value iteration algorithm...\n');
fprintf('Iteration #');
ll = 0;

converged = 0;
J = zeros(1, nXX);

for k = 1 : dpf.max_iter 
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);
    
    J_old = J;
          
    [J_min, J_min_idx] = min(dpf.stage_cost_fn(X, U, k, dpf.T_ocp) + ...
                             reshape(J_old(next_ind), nUU, nXX), [], 1);
    
    descendant_matrix = next_ind(fastsub2ind2([nUU nXX], J_min_idx, 1:nXX))';
    
    [b{:}]  = ind2sub(nU, J_min_idx);
    for i = 1 : n_inputs
        U_star_matrix{i} = b{i};    
    end

    J = J_min;    
    
    delta_J(k) = norm(J-J_old);
    if  delta_J(k) < tol
        converged = 1;
        break;
    end
    
end

fprintf('\nComplete!\n')

if converged == 1
    fprintf('Converging after %i iterations with a tollerance of %f!\n', k, tol);    
else
    fprintf('Still NOT converging after = %i iterations!\n', k);
    fprintf(['  Tracing may fail!\n' ...
             '  Consider increasing the number of maximum iterations.\n']);
end

% Store the results
dpf.n_iter             = k;
dpf.descendant_matrix  = descendant_matrix;
dpf.U_star_matrix      = U_star_matrix;

% Additional information that might be still needed
dpf.n_states = n_states;
dpf.n_inputs = n_inputs;
dpf.nX       = nX;
dpf.nU       = nU;
dpf.lb       = lb;
dpf.ub       = ub;
dpf.nUU      = nUU;
dpf.nXX      = nXX;

dpf.delta_J  = delta_J;
dpf.method   = 'vi';

end
%------------- END OF CODE --------------
