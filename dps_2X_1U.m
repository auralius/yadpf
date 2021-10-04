% Dynamic programming solver for a system with one input variable and one 
% state variable
%
% function dps = dps_2X_1U(X1, ...
%                X2, ...
%                U, ...
%                n_horizon, ...
%                state_update_fn, ...
%                stage_cost_fn, ...
%                terminal_cost_fn)
%
% Arguments:
%
%     X1                = discretized first state (nX1 x 1)
%     X2                = discretized first state (nX2 x 1)
%     U                 = discretized input (nU x 1)
%     n_horizon         = horizon length (a positive integer)
%     state_update_fn   = user defined system model
%     stage_cost_fn     = user defined stage cost function
%     terminal_cost_fn  = user defined terminal cost function
%
function dps = dps_2X_1U(X1, ...
    X2, ...
    U, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost_fn, ...
    terminal_cost_fn)

% Frequently used parameters/variables
nU    = length(U);
nX1   = length(X1);
nX2   = length(X2);
N     = nX1*nX2;
lb    = [min(X1) min(X2)];
ub    = [max(X1) max(X2)];

% Where to keep the results?
J                = ones(N,  n_horizon).*inf; % Cost matrix
U_star_matrix    = zeros(N, n_horizon);      % Store the optimal inputs
decendent_matrix = zeros(N, n_horizon);      % Store the optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', N);
fprintf('Input    : %i nodes\n', nU);
fprintf('Running backward dynamic programming algorithm...\n');

tic

% The terminal cost is only a function of the state variables
[r, c] = ind2sub([nX1 nX2], 1:N);
J(:, n_horizon) = terminal_cost_fn(X1(r), X2(c));

% Precompute for all nodes and all inputs
i = repmat((1:N)', 1,nU);
[r, c] = ind2sub([nX1 nX2], i);
[x1_next, x2_next] = state_update_fn(X1(r), X2(c), ...
    repmat(U',N,1));

% Bound the states within the minimum and maximum values
x1_next = min(max(x1_next, ...
    repmat(lb(1),N,nU)), repmat(ub(1),N,nU));
x2_next = min(max(x2_next, ...
    repmat(lb(2),N,nU)), repmat(ub(2),N,nU));

r = snap(x1_next, repmat(lb(1),N,nU), repmat(ub(1),N,nU), ...
    repmat(nX1-1,N,nU));
c = snap(x2_next, repmat(lb(2),N,nU), repmat(ub(2),N,nU), ...
    repmat(nX2-1,N,nU));

ind = sub2ind([nX1 nX2], r, c);

% Stage-wise iteration
fprintf('Stage-');
for k = n_horizon-1 : -1 : 1
    ll = fprintf('%i',k);
    
    J_ = stage_cost_fn(X1(r), X2(c), repmat(U',N,1), k) + ...
        reshape(J(ind,k+1),N,nU);
    
    [J_min, J_min_idx] = min(J_, [], 2);
    
    % I have tried to vectorize this section, it actually becomes
    % slower
    for q = 1:N
        decendent_matrix(q,k) = ind(q,J_min_idx(q));
    end
    
    U_star_matrix(:,k) = U(J_min_idx);
    J(:,k) = J_min;
    
    fprintf(repmat('\b',1,ll));
end

fprintf('1\nCompleted\n');

% Store the results
dps.J = J;
dps.decendent_matrix = decendent_matrix;
dps.U_star_matrix = U_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X1        = X1;
dps.X2        = X2;
dps.U         = U;

toc

end
