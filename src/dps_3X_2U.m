% Dynamic programming solver for a system with two input variables and 
% three state variables
%
% function dps = dps_3X_2U(X1, ...
%                X2, ...
%                C3, ...
%                U1, ...
%                U2, ...
%                n_horizon, ...
%                state_update_fn, ...
%                stage_cost_fn, ...
%                terminal_cost_fn)
%
% Arguments:
%
%     X1                = discretized first state (nX1 x 1)
%     X2                = discretized second state (nX2 x 1)
%     X3                = discretized third state (nX3 x 1)
%     U1                = discretized first input (nU1 x 1)
%     U2                = discretized second input (nU2 x 1)
%     n_horizon         = horizon length (a positive integer)
%     state_update_fn   = user defined system model
%     stage_cost_fn     = user defined stage cost function
%     terminal_cost_fn  = user defined terminal cost function
%
function dps = dps_3X_2U(X1, ...
    X2, ...
    X3, ...
    U1, ...
    U2, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost_fn, ...
    terminal_cost_fn)

% Frequently used parameters/variables
nU1   = length(U1);
nU2   = length(U2);
nU    = nU1*nU2;
nX1   = length(X1);
nX2   = length(X2);
nX3   = length(X3);
nX    = nX1*nX2*nX3;
lb    = [min(X1) min(X2) min(X3)];
ub    = [max(X1) max(X2) max(X3)];

% Where to keep the results?
U1_star_matrix    = zeros(nX, n_horizon, 'uint32');      % Optimal inputs
U2_star_matrix    = zeros(nX, n_horizon, 'uint32');      % Optimal inputs
descendant_matrix = zeros(nX, n_horizon, 'uint32');      % Optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Pre-calculation, please wait...\n')

% The terminal cost is only a function of the state variables
[r, c, p] = ind2sub([nX1 nX2 nX3], 1:nX);
J = terminal_cost_fn(X1(r), X2(c), X3(p));

% Precompute for all nodes and all inputs
i = fastrepcolvec((1:nX)',nU);
[r, c, p] = ind2sub([nX1 nX2 nX3], i);
i = fastreprowvec(1:nU, nX);
[ru, cu] = ind2sub([nU1 nU2], i);
clear i;

[x1_next, x2_next, x3_next] = state_update_fn(X1(r), X2(c), X3(p),...
    U1(ru), U2(cu));

% Bound the states within the minimum and maximum values
x1_next = min(max(x1_next, lb(1)), ub(1));
x2_next = min(max(x2_next, lb(2)), ub(2));
x3_next = min(max(x3_next, lb(3)), ub(3));

r = snap(x1_next, fastsca2mat(lb(1),nX,nU), fastsca2mat(ub(1),nX,nU), ...
    fastsca2mat(nX1,nX,nU));
c = snap(x2_next, fastsca2mat(lb(2),nX,nU), fastsca2mat(ub(2),nX,nU), ...
    fastsca2mat(nX2,nX,nU));
p = snap(x3_next, fastsca2mat(lb(3),nX,nU), fastsca2mat(ub(3),nX,nU), ...
    fastsca2mat(nX3,nX,nU));

ind = fastsub2ind3([nX1 nX2 nX3], r, c, p);

fprintf('Complete!\n');

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);    
    
    J_old = J;
    [J_min, J_min_idx] = min(stage_cost_fn(X1(r), X2(c), X3(p), ...
                                           U1(ru), U2(cu), k) + ...
                             reshape(J_old(ind),nX,nU), [], 2);
    
    descendant_matrix(:,k) = ind(fastsub2ind2([nX nU],(1:nX)',J_min_idx));
    
    [a, b] = ind2sub([nU1 nU2], J_min_idx);
    U1_star_matrix(:,k) = a;
    U2_star_matrix(:,k) = b;
    J = J_min;    
end

fprintf('\nComplete!\n');

% Store the results
dps.descendant_matrix = descendant_matrix;
dps.U1_star_matrix    = U1_star_matrix;
dps.U2_star_matrix    = U2_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X1        = X1;
dps.X2        = X2;
dps.X3        = X3;
dps.U1        = U1;
dps.U2        = U2;

dps.n_states  = 3;
dps.n_inputs  = 2;

end