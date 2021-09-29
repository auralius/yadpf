function dps = dpsf_1X_1U(X, ...
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

fprintf('Running forward dynamic programming algorithm...\n');  
tic

% The stage cost is a function of both the state variables and the input
for k = 1 : n_horizon  
    ll = fprintf('Stage-%i',k);    
    
    for i = 1 :  nX      % 1 state variable        
        x_next = state_update_fn(repmat(X(i),nU,1), U);
        
        % Bound the states within the minimum and maximum values
        x_next_post_boundary = min(max(x_next, repmat(lb,nU,1)), repmat(ub,nU,1));
        
        is_infeasible = (x_next~=x_next_post_boundary);
        
        ind = snap(x_next_post_boundary, ...
            repmat(lb,nU,1), repmat(ub,nU,1), repmat(nX-1,nU,1));
                
        J_ = stage_cost(repmat(X(i),nU,1), U) ...
            + J(ind,k+1) + is_infeasible .* 1e6;        
        [J_min, J_min_idx] = min(J_);
        
        decendent_matrix(i,k) = ind(J_min_idx);
        U_star_matrix(i,k) = U(J_min_idx);
        J(i,k) = J_min;           
    end    
    
    fprintf(repmat('\b',1,ll));
end

% The terminal cost is only a function of the state variables 
J(:, n_horizon+1) = terminal_cost_fn(X);

fprintf('Completed\n');

% Store the results
dps.J = J;
dps.decendent_matrix = decendent_matrix;
dps.U_star_matrix = U_star_matrix;

% Additional information that might be still needed
dps.n_horizon           = n_horizon;
dps.X                   = X;
dps.U                   = U;

toc
end
