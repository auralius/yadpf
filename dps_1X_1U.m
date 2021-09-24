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

J                = ones (nX, n_horizon).*inf; % Cost matrix
U_star_matrix    = zeros(nX, n_horizon);      % Store the optimal inputs
decendent_matrix = zeros(nX, n_horizon);      % Store the optimal next state

% The terminal cost is only a function of the state variables
for i = 1 : nX
    J(i, n_horizon+1) = terminal_cost_fn(X(i));
end

% The stage cost is a function of both the state variables and the input
for k = n_horizon : -1 : 1
    for i = 1 :  nX     % 1 state variable
        for j = 1 : nU  % 1 input
            x_next = state_update_fn(X(i), U(j));
            
            % Bound the states within the minimum and maximum values
            x_next = min(max(x_next, lb), ub);
            
            n = snap(x_next, lb, ub, nX-1);
                                       
            J_ = stage_cost(X(i), U(j)) + J(n,k+1);
            
            if  (J_ < J(i,k)) && (n ~= i)
                decendent_matrix(i,k) = n;
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
dps.X        = X;
dps.U         = U;

end
