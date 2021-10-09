% Dynamic programming solver for a system with three input variable and one 
% state variable
%
% function dps = dps_3X_1U(X1, ...
%                X2, ...
%                C3, ...
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
%     X3                = discretized first state (nX3 x 1)
%     U                 = discretized input (nU x 1)
%     n_horizon         = horizon length (a positive integer)
%     state_update_fn   = user defined system model
%     stage_cost_fn     = user defined stage cost function
%     terminal_cost_fn  = user defined terminal cost function
%
function dps = dps_3X_1U(X1, ...
    X2, ...
    X3, ...
    U, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost_fn, ...
    terminal_cost_fn)

% Frequently used parameters/variables
nU    = length(U);
nX1   = length(X1);
nX2   = length(X2);
nX3   = length(X3);
nX    = nX1*nX2*nX3;
lb    = [min(X1) min(X2) min(X3)];
ub    = [max(X1) max(X2) max(X3)];

% Where to keep the results?
J                 = ones(nX,  n_horizon).*inf; % Cost matrix
U_star_matrix     = zeros(nX, n_horizon);      % Store the optimal inputs
descendant_matrix = zeros(nX, n_horizon);      % Store the optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Pre-calculation, please wait...\n')

% The terminal cost is only a function of the state variables
[r, c, p] = ind2sub([nX1 nX2 nX3], 1:nX);
J(:, n_horizon) = terminal_cost_fn(X1(r), X2(c), X3(p));

% Precompute for all nodes and all inputs
i = repmat((1:nX)', 1,nU);
[r, c, p] = ind2sub([nX1 nX2 nX3], i);
[x1_next, x2_next, x3_next] = state_update_fn(X1(r), X2(c), X3(p),...
    repmat(U',nX,1));

% Bound the states within the minimum and maximum values
x1_next_post_boundary = min(max(x1_next, lb(1)), ub(1));
x2_next_post_boundary = min(max(x2_next, lb(2)), ub(2));
x3_next_post_boundary = min(max(x3_next, lb(3)), ub(3));

is_infeasible = (x1_next~=x1_next_post_boundary) + ...
    (x2_next~=x2_next_post_boundary) + ...
    (x3_next~=x3_next_post_boundary);

r = snap(x1_next_post_boundary, repmat(lb(1),nX,nU), repmat(ub(1),nX,nU), ...
    repmat(nX1,nX,nU));
c = snap(x2_next_post_boundary, repmat(lb(2),nX,nU), repmat(ub(2),nX,nU), ...
    repmat(nX2,nX,nU));
p = snap(x3_next_post_boundary, repmat(lb(3),nX,nU), repmat(ub(3),nX,nU), ...
    repmat(nX3,nX,nU));

ind = sub2ind([nX1 nX2 nX3], r, c, p);

fprintf('Complete!\n');

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);    
    
    J_ = stage_cost_fn(X1(r), X2(c), X3(p), repmat(U',nX,1), k) + ...
        reshape(J(ind,k+1),nX,nU) + is_infeasible .* 1e6;
    
    [J_min, J_min_idx] = min(J_, [], 2);
    
    descendant_matrix(:,k) = ind(sub2ind([nX nU],(1:nX)',J_min_idx));
    
    U_star_matrix(:,k) = U(J_min_idx);
    J(:,k) = J_min;    
end

fprintf('\nComplete!\n');

% Store the results
dps.J                 = J;
dps.descendant_matrix = descendant_matrix;
dps.U_star_matrix     = U_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X1                  = X1;
dps.X2                  = X2;
dps.X3                  = X3;
dps.U                   = U;
dps.state_update_fn     = state_update_fn;
end
