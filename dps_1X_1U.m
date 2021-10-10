% Dynamic programming solver for a system with one input variable and one 
% state variable
%
% function dps = dps_1X_1U(X, ...
%                U, ...
%                n_horizon, ...
%                state_update_fn, ...
%                stage_cost_fn, ...
%                terminal_cost_fn)
%
% Arguments:
%
%     X                 = discretized state (nX x 1)
%     U                 = discretized input (nU x 1)
%     n_horizon         = horizon length (a positive integer)
%     state_update_fn   = user defined system model
%     stage_cost_fn     = user defined stage cost function
%     terminal_cost_fn  = user defined terminal cost function
%
function dps = dps_1X_1U(X, ...
    U, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost_fn, ...
    terminal_cost_fn)

nU = length(U);
nX = length(X);
lb = min(X);
ub = max(X);

J                 = ones(nX, n_horizon).*inf;  % Cost matrix
U_star_matrix     = zeros(nX, n_horizon);      % Store the optimal inputs
descendant_matrix = zeros(nX, n_horizon);      % Store the optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Pre-calculation, please wait...\n')

% The terminal cost is only a function of the state variables
J(:, n_horizon) = terminal_cost_fn(X);

% Precompute for all nodes and all inputs
i = repmat((1:nX)', 1,nU);
x_next = state_update_fn(X(i), repmat(U',nX,1));

% Bound the states within the minimum and maximum values
x_next_post_boundary = min(max(x_next, lb), ub);

ind = snap(x_next_post_boundary, ...
    repmat(lb,nX,nU), repmat(ub,nX,nU), repmat(nX,nX,nU));

fprintf('Completed!\n');

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);
    
    J_ = stage_cost_fn(X(i), repmat(U',nX,1), k) ...
        + reshape(J(ind,k+1),nX,nU);
    
    [J_min, J_min_idx] = min(J_, [], 2); % Row-wise
    
    descendant_matrix(:,k) = ind(sub2ind([nX nU],(1:nX)', J_min_idx));
        
    U_star_matrix(:,k) = U(J_min_idx);
    J(:,k) = J_min;
end

fprintf('\nCompleted!\n');

% Store the results
dps.J = J;
dps.descendant_matrix = descendant_matrix;
dps.U_star_matrix = U_star_matrix;

% Additional information that might be still needed
dps.n_horizon           = n_horizon;
dps.X                   = X;
dps.U                   = U;

end
