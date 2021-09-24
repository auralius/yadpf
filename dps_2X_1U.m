function dps = dps_2X_1U(X1, ...                                     
                         X2, ...
                         U, ...
                         n_horizon, ...
                         state_update_fn, ...
                         stage_cost_fn, ...
                         terminal_cost_fn)
                     
nU    = length(U);
nX1   = length(X1);
nX2   = length(X2);
N     = nX1*nX2;

lb = [min(X1) min(X2)];
ub = [max(X1) max(X2)];

J                = ones(N,  n_horizon).*inf; % Cost matrix
U_star_matrix    = zeros(N, n_horizon);      % Store the optimal inputs
decendent_matrix = zeros(N, n_horizon);      % Store the optimal next state

% The terminal cost is only a function of the state variables
for i = 1 : N
    [x1, x2] = ind2sub([nX1 nX2], i);
    
    % Convert index back to actual value
    x1 = X1(x1);
    x2 = X2(x2);
    
    J(i, n_horizon+1) = terminal_cost_fn(x1, x2);
end

% The stage cost is a function of both the state variables and the input
for k = n_horizon : -1 : 1
    for i = 1 :  N                              % 2 state variables
        [x1, x2] = ind2sub([nX1 nX2], i);

        % Convert index back to actual value
        x1 = X1(x1);
        x2 = X2(x2);

        for j = 1 : nU  % 1 input
            [x1_next, x2_next] = state_update_fn(x1, x2, U(j));
            
            % Bound the states within the minimum and maximum values
            x1_next = min(max(x1_next, lb(1)), ub(1));
            x2_next = min(max(x2_next, lb(2)), ub(2));
            
            r = snap(x1_next, lb(1), ub(1), nX1-1);
            c = snap(x2_next, lb(2), ub(2), nX2-1);
                           
            ind = sub2ind([nX1 nX2], r, c);

            J_ = stage_cost_fn(X1, X2, U(j)) + J(ind,k+1);
            
            if  (J_ < J(i,k)) && (ind ~= i)
                decendent_matrix(i,k) = ind;
                U_star_matrix(i,k) = U(j);
                J(i,k) = J_;
            end
        end
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

end
