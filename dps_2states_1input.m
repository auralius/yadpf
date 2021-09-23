function dps = dps_2states_1input(states, input, n_horizon, state_update_fn, stage_cost, terminal_cost)

dps.n_states         = length(states);
dps.states           = states;
dps.input            = input;
dps.state_update_fn  = state_update_fn;
dps.stage_cost       = stage_cost;
dps.terminal_cost    = terminal_cost;
dps.n_horizon        = n_horizon;

X1    = dps.states(1).elements;
X2    = dps.states(2).elements;
U     = dps.input.elements;
nX1   = length(dps.states(1).elements);
nX2   = length(dps.states(2).elements);
N     = nX1*nX2;

dps.J                = ones(N, dps.n_horizon).*inf; % Cost matrix
dps.U_star_matrix    = zeros(N, dps.n_horizon);     % Store the optimal inputs
dps.decendent_matrix = zeros(N, dps.n_horizon);     % Store the optimal next state


% The terminal cost is only a function of the state variables
for i = 1 : nX1*nX2
    [x1,x2] = ind2sub([nX1 nX2], i);
    
    % Convert index back to actual value
    x1 = X1(x1);
    x2 = X2(x2);
    
    dps.J(i, dps.n_horizon+1) = dps.terminal_cost([x1 x2]);
end

% The stage cost is a function of both the state variables and the input
for k = dps.n_horizon : -1 : 1
    for i = 1 :  N                              % 2 state variables
        for j = 1 : length(dps.input.elements)  % 1 input
            [x1, x2] = ind2sub([nX1 nX2], i);
            
            % Convert index back to actual value
            x1 = X1(x1);
            x2 = X2(x2);
           
            next_states = state_update_fn([x1 x2], U(j));
            
            % Bound the states within the minimum and maximum values
            next_states(1) = max(next_states(1), dps.states(1).lb);
            next_states(2) = max(next_states(2), dps.states(2).lb);
            next_states(1) = min(next_states(1), dps.states(1).ub);
            next_states(2) = min(next_states(2), dps.states(2).ub);
            
            x1_next_idx = snap(next_states(1), dps.states(1).lb, dps.states(1).ub, nX1-1);
            x2_next_idx = snap(next_states(2), dps.states(2).lb, dps.states(2).ub, nX2-1);
            ind = sub2ind([nX1 nX2], x1_next_idx, x2_next_idx);

            J_ = stage_cost([X1 X2], U(j)) + dps.J(ind,k+1);
            
            if  (J_ < dps.J(i,k)) && (ind ~= i)
                dps.decendent_matrix(i,k) = ind;
                dps.U_star_matrix(i,k) = U(j);
                dps.J(i,k) = J_;
            end
        end
    end
end

end
