% Dynamic programming solver for a system with two input variable and two 
% state variables
%
% function dps = dps_2X_2U(X1, ...
%                X2, ...
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
%     X2                = discretized first state (nX2 x 1)
%     U1                = discretized input (nU1 x 1)
%     U2                = discretized input (nU2 x 1)
%     n_horizon         = horizon length (a positive integer)
%     state_update_fn   = user defined system model
%     stage_cost_fn     = user defined stage cost function
%     terminal_cost_fn  = user defined terminal cost function
%
function dps = dps_2X_2U(X1, ...
    X2, ...
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
nX    = nX1*nX2;
lb    = [min(X1) min(X2)];
ub    = [max(X1) max(X2)];

% Where to keep the results?
J                 = ones(nX,  n_horizon).*inf; % Cost matrix
U1_star_matrix    = zeros(nX, n_horizon);      % Store the optimal inputs
U2_star_matrix    = zeros(nX, n_horizon);      % Store the optimal inputs
decendent_matrix  = zeros(nX, n_horizon);      % Store the optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Running backward dynamic programming algorithm...\n');

tic

% The terminal cost is only a function of the state variables
[rx, cx] = ind2sub([nX1 nX2], 1:nX);
J(:, n_horizon) = terminal_cost_fn(X1(rx), X2(cx));

% Precompute for all nodes and all inputs
i = repmat((1:nX)', 1, nU);
j = repmat((1:nU) , nX, 1);
[rx, cx] = ind2sub([nX1 nX2], i);
[ru, cu] = ind2sub([nU1 nU2], j);

[x1_next, x2_next] = state_update_fn(X1(rx), X2(cx), ...
    U1(ru), U2(cu));

% Bound the states within the minimum and maximum values
x1_next = min(max(x1_next, ...
    repmat(lb(1),nX,nU)), repmat(ub(1),nX,nU));
x2_next = min(max(x2_next, ...
    repmat(lb(2),nX,nU)), repmat(ub(2),nX,nU));

rx = snap(x1_next, repmat(lb(1),nX,nU), repmat(ub(1),nX,nU), ...
    repmat(nX1-1,nX,nU));
cx = snap(x2_next, repmat(lb(2),nX,nU), repmat(ub(2),nX,nU), ...
    repmat(nX2-1,nX,nU));

ind = sub2ind([nX1 nX2], rx, cx);

% Stage-wise iteration
fprintf('Stage-');
for k = n_horizon-1 : -1 : 1
    ll = fprintf('%i',k);
    
    J_ = stage_cost_fn(X1(rx), X2(cx), U1(ru), U2(cu), k) + ...
        reshape(J(ind,k+1),nX,nU);
    
    [J_min, J_min_idx] = min(J_, [], 2);
    
    % I have tried to vectorize this section, it actually becomes
    % slower
    for q = 1:nX
        decendent_matrix(q,k) = ind(q,J_min_idx(q));
    end
    
    [a, b] = ind2sub([nU1 nU2], J_min_idx);
    U1_star_matrix(:,k) = U1(a);
    U2_star_matrix(:,k) = U2(b);
    J(:,k) = J_min;
    
    fprintf(repmat('\b',1,ll));
end

fprintf('1\nCompleted\n');

% Store the results
dps.J = J;
dps.decendent_matrix = decendent_matrix;
dps.U1_star_matrix = U1_star_matrix;
dps.U2_star_matrix = U2_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X1        = X1;
dps.X2        = X2;
dps.U1        = U1;
dps.U2        = U2;

toc

end
