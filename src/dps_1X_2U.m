% Dynamic programming solver for a system with two input variables and one 
% state variable
%
% function dps = dps_1X_2U(X, ...
%                U1, ...
%                U2, ...
%                n_horizon, ...
%                state_update_fn, ...
%                stage_cost_fn, ...
%                terminal_cost_fn, ..
%                T_ocp, ...
%                T_dyn)
%
% Arguments:
%
%     X                 = discretized state (nX x 1)
%     U1                = discretized input 1 (nU1 x 1)
%     U2                = discretized input 2 (nU2 x 1)
%     n_horizon         = horizon length (a positive integer)
%     state_update_fn   = user defined system model
%     stage_cost_fn     = user defined stage cost function
%     terminal_cost_fn  = user defined terminal cost function
%     T_ocp             = sampling time for the optimal control problem
%     T_dyn             = sampling time for the system's dynamic simulation
%
%
function dps = dps_1X_2U(X, ...
    U1, ...
    U2, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost_fn, ...
    terminal_cost_fn, ...
    T_ocp, ...
    T_dyn)                         

% Check the arguments
if nargin < 8
    T_ocp = 1;
    T_dyn = T_ocp;
elseif nargin < 9
    T_dyn = T_ocp;
end 

if T_dyn > T_ocp
    error('Time step for simulating the system must be less or equal than the time step for the OCP discretization!')
end

nU1 = length(U1);
nU2 = length(U2);
nU  = nU1*nU2;
nX  = length(X);
lb  = min(X);
ub  = max(X);

U1_star_matrix    = zeros(n_horizon, nX, 'uint32'); % Optimal input 1
U2_star_matrix    = zeros(n_horizon, nX, 'uint32'); % Optimal input 2
descendant_matrix = zeros(n_horizon, nX, 'uint32'); % Optimal next-state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Calculating terminal cost...\n')

% The terminal cost is only a function of the state variables
J = terminal_cost_fn(X);

% Precompute for all nodes and all inputs
i = fastreprowvec(1:nX,nU);
j = fastrepcolvec((1:nU)', nX);
[ru, cu] = ind2sub([nU1 nU2], j);
clear j;

% Simulate the system with sampling period of Tdyn
fprintf('Simulating the system one step ahead...\n');
fprintf('Progress: ')

n_iter = T_ocp/T_dyn; % Default time step for the dynamic simulation is 0.01[s]
x_next = X(i);

ll = 0;
for k = 1 : 1 : n_iter
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',k/n_iter*100);
    x_next = state_update_fn(X(i), U1(ru), U2(cu), T_dyn);
end

% Bound the states within the minimum and maximum values
x_next = min(max(x_next, lb), ub);

next_ind = snap(x_next, ...
    fastsca2mat(lb,nU,nX), fastsca2mat(ub,nU,nX), fastsca2mat(nX,nU,nX));

fprintf('\nCompleted!\n');

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);
    
    J_old = J;
    
    [J_min, J_min_idx] = min(stage_cost_fn(X(i), U1(ru), U2(cu), k, T_ocp) ...
                             + reshape(J_old(next_ind),nU,nX), [], 1);
    
    descendant_matrix(k,:) = next_ind(fastsub2ind2([nU nX], J_min_idx, 1:nX))';
    
    [a, b] = ind2sub([nU1 nU2], J_min_idx);
    U1_star_matrix(k,:) = a;
    U2_star_matrix(k,:) = b;
       
    J = J_min;
end

fprintf('\nCompleted!\n');

% Store the results
dps.descendant_matrix = descendant_matrix;
dps.U1_star_matrix = U1_star_matrix;
dps.U2_star_matrix = U2_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X         = X;
dps.U1        = U1;
dps.U2        = U2;
dps.nX        = nX;
dps.nU        = nU;

dps.lb        = lb;
dps.ub        = ub;

dps.n_states  = 1;
dps.n_inputs  = 2;

dps.T_ocp = T_ocp;
dps.T_dyn = T_dyn;

dps.state_update_fn = state_update_fn;


end
