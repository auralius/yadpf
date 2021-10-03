function dps = dps_1X_1U(X, ...
    U, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost, ...
    terminal_cost_fn)

nU = length(U);
nX = length(X);
lb = min(X);
ub = max(X);

J                    = ones(nX, n_horizon).*inf;  % Cost matrix
U_star_matrix        = zeros(nX, n_horizon);      % Store the optimal inputs
decendent_matrix     = zeros(nX, n_horizon);      % Store the optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Running backward dynamic programming algorithm...\n');

tic

% The terminal cost is only a function of the state variables
J(:, n_horizon) = terminal_cost_fn(X);

% Precompute for all nodes and all inputs
i = repmat((1:nX)', 1,nU);
x_next = state_update_fn(X(i), repmat(U',nX,1));

% Bound the states within the minimum and maximum values
x_next_post_boundary = min(max(x_next, repmat(lb,nX,nU)), ...
    repmat(ub,nX,nU));

is_infeasible = (x_next~=x_next_post_boundary);

ind = snap(x_next_post_boundary, ...
    repmat(lb,nX,nU), repmat(ub,nX,nU), repmat(nX-1,nX,nU));

% Stage-wise iteration
fprintf('Stage-');
for k = n_horizon-1 : -1 : 1
    ll = fprintf('%i',k);
    
    J_ = stage_cost(X(i), repmat(U',nX,1), k) ...
        + reshape(J(ind,k+1),nX,nU) + is_infeasible .* 1e6;
    
    [J_min, J_min_idx] = min(J_, [], 2); % Row-wise
    
    % I have tried to vectorize this section, it actually becomes
    % slower
    for q = 1:nX
        decendent_matrix(q,k) = ind(q,J_min_idx(q));
    end
    
    U_star_matrix(:,k) = U(J_min_idx);
    J(:,k) = J_min;
    
    fprintf(repmat('\b',1,ll));
end

fprintf('1\nCompleted\n');

toc

% Store the results
dps.J = J;
dps.decendent_matrix = decendent_matrix;
dps.U_star_matrix = U_star_matrix;

% Additional information that might be still needed
dps.n_horizon           = n_horizon;
dps.X                   = X;
dps.U                   = U;

end
