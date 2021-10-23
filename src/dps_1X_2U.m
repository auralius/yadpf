% Dynamic programming solver for a system with two input variables and one 
% state variable
%
% function dps = dps_1X_2U(X, ...
%                U1, ...
%                U2, ...
%                n_horizon, ...
%                state_update_fn, ...
%                stage_cost_fn, ...
%                terminal_cost_fn)
%
% Arguments:
%
%     X                 = discretized state (nX x 1)
%     U1                = discretized input 1 (nU1 x 1)
%     U2                = discretized input 2 (nU2 x 1)
%     n_horizon         = horizon length (a positive integer)
%     state_update_fn   = user defined system model
%     stage_cost_fn     = user defined stage cost function
%     terminal_cost_fn  = user defined terminal cost function
%
function dps = dps_1X_2U(X, ...
    U1, ...
    U2, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost_fn, ...
    terminal_cost_fn)

nU1 = length(U1);
nU2 = length(U2);
nU  = nU1*nU2;
nX  = length(X);
lb  = min(X);
ub  = max(X);

U1_star_matrix    = zeros(nX, n_horizon, 'uint32'); % Optimal input 1
U2_star_matrix    = zeros(nX, n_horizon, 'uint32'); % Optimal input 2
descendant_matrix = zeros(nX, n_horizon, 'uint32'); % Optimal next-state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Pre-calculation, please wait...\n')

% The terminal cost is only a function of the state variables
J = terminal_cost_fn(X);

% Precompute for all nodes and all inputs
i = fastrepcolvec((1:nX)',nU);
j = fastreprowvec(1:nU, nX);
[ru, cu] = ind2sub([nU1 nU2], j);
clear j;

x_next = state_update_fn(X(i), U1(ru), U2(cu));

% Bound the states within the minimum and maximum values
x_next = min(max(x_next, lb), ub);

ind = snap(x_next, ...
    fastsca2mat(lb,nX,nU), fastsca2mat(ub,nX,nU), fastsca2mat(nX,nX,nU));

fprintf('Completed!\n');

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);
    
    J_old = J;
    
    [J_min, J_min_idx] = min(stage_cost_fn(X(i), U1(ru), U2(cu), k) ...
                             + reshape(J_old(ind),nX,nU), [], 2);
    
    descendant_matrix(:,k) = ind(fastsub2ind2([nX nU],(1:nX)', J_min_idx));
    
    [a, b] = ind2sub([nU1 nU2], J_min_idx);
    U1_star_matrix(:,k) = a;
    U2_star_matrix(:,k) = b;
       
    J = J_min;
end

fprintf('\nCompleted!\n');

% Store the results
dps.descendant_matrix = descendant_matrix;
dps.U1_star_matrix = U1_star_matrix;
dps.U2_star_matrix = U2_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X         = X;
dps.U1        = U1;
dps.U2        = U2;
dps.nX        = nX;
dps.nU        = nU;

dps.n_states  = 1;
dps.n_inputs  = 2;


end
