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

% The terminal cost is only a function of the state variables
tic
fprintf('Terminal stage...\n');
[r, c] = ind2sub([nX1 nX2], 1:N);
J(:, n_horizon) = terminal_cost_fn(X1(r), X2(c));

% The stage cost is a function of both the state variables and the input
for k = n_horizon-1 : -1 : 1
    fprintf('Stage-%i...\n',k);
    for i = 1 :  N
        [r, c] = ind2sub([nX1 nX2], i);
        [x1_next, x2_next] = state_update_fn(repmat(X1(r),nU,1), ...
            repmat(X2(c),nU,1), ...
            U);
        
        % Bound the states within the minimum and maximum values
        x1_next = min(max(x1_next, ...
            repmat(lb(1),nU,1)), repmat(ub(1),nU,1));
        x2_next = min(max(x2_next, ...
            repmat(lb(2),nU,1)), repmat(ub(2),nU,1));
        
        r = snap(x1_next, repmat(lb(1),nU,1), repmat(ub(1),nU,1), ...
            repmat(nX1-1,nU,1));
        c = snap(x2_next, repmat(lb(2),nU,1), repmat(ub(2),nU,1), ...
            repmat(nX2-1,nU,1));
        
        ind = sub2ind([nX1 nX2], r, c);
        
        [J_min, J_min_idx] = min(stage_cost_fn(X1, X2, U) + J(ind,k+1));
        
        decendent_matrix(i,k) = ind(J_min_idx);
        U_star_matrix(i,k) = U(J_min_idx);
        J(i,k) = J_min;
    end
end

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
