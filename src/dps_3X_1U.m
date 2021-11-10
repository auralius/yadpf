% Dynamic programming solver for a system with one input variable and 
% three state variables
%
% function dps = dps_3X_1U(X1, ...
%                X2, ...
%                X3, ...
%                U, ...
%                n_horizon, ...
%                state_update_fn, ...
%                stage_cost_fn, ...
%                terminal_cost_fn, ...
%                T_ocp, ...
%                T_dyn)
%
% Arguments:
%
%    X1                = discretized first state (nX1 x 1)
%    X2                = discretized first state (nX2 x 1)
%    X3                = discretized first state (nX3 x 1)
%    U                 = discretized input (nU x 1)
%    n_horizon         = horizon length (a positive integer)
%    state_update_fn   = user defined system model
%    stage_cost_fn     = user defined stage cost function
%    terminal_cost_fn  = user defined terminal cost function
%    T_ocp             = sampling time for the optimal control problem
%    T_dyn             = sampling time for the system's dynamic simulation
%
%
function dps = dps_3X_1U(X1, ...
    X2, ...
    X3, ...
    U, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost_fn, ...
    terminal_cost_fn, ...
    T_ocp, ...
    T_dyn)

% Check the arguments
if nargin < 9
    T_ocp = 1;
    T_dyn = T_ocp;
elseif nargin < 10
    T_dyn = T_ocp;
end 

if T_dyn > T_ocp
    error('Time step for simulating the system must be less or equal than the time step for the OCP discretization!')
end

% Frequently used parameters/variables
nU    = length(U);
nX1   = length(X1);
nX2   = length(X2);
nX3   = length(X3);
nX    = nX1*nX2*nX3;
lb    = [min(X1) min(X2) min(X3)];
ub    = [max(X1) max(X2) max(X3)];

% Where to keep the results?
U_star_matrix     = zeros(n_horizon, nX, 'uint32');   % Optimal inputs
descendant_matrix = zeros(n_horizon, nX, 'uint32');   % Optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Calculating terminal cost...\n')

% The terminal cost is only a function of the state variables
[r, c, p] = ind2sub([nX1 nX2 nX3], 1:nX);
J = terminal_cost_fn(X1(r), X2(c), X3(p));

% Precompute for all nodes and all inputs
i = fastreprowvec(1:nX, nU);
[r, c, p] = ind2sub([nX1 nX2 nX3], i);
clear i;

% Simulate the system with sampling period of Tdyn
fprintf('Simulating the system one step ahead...\n');
fprintf('Progress: ');

n_iter = T_ocp/T_dyn; 
x1_next = X1(r);
x2_next = X2(c);
x3_next = X3(p);
UU = fastrepcolvec(U',nX);

ll = 0;
for k = 1 : 1 : n_iter
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',k/n_iter*100);
    [x1_next, x2_next, x3_next] = ...
        state_update_fn(x1_next, x2_next, x3_next, UU, T_dyn);    
end

clear UU;

% Bound the states within the minimum and maximum values
x1_next = min(max(x1_next, lb(1)), ub(1));
x2_next = min(max(x2_next, lb(2)), ub(2));
x3_next = min(max(x3_next, lb(3)), ub(3));

r = snap(x1_next, fastsca2mat(lb(1),nU,nX), fastsca2mat(ub(1),nU,nX), ...
    fastsca2mat(nX1,nU,nX));
c = snap(x2_next, fastsca2mat(lb(2),nU,nX), fastsca2mat(ub(2),nU,nX), ...
    fastsca2mat(nX2,nU,nX));
p = snap(x3_next, fastsca2mat(lb(3),nU,nX), fastsca2mat(ub(3),nU,nX), ...
    fastsca2mat(nX3,nU,nX));

clear x1_next x2_next x3 next;

next_ind = fastsub2ind3([nX1 nX2 nX3], r, c, p);

% Stage-wise iteration
fprintf('\nRunning backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);    
    
    J_old = J;
    [J_min, J_min_idx] = min(stage_cost_fn(X1(r), X2(c), X3(p), ...
        fastrepcolvec(U',nX), k, T_ocp) + reshape(J_old(next_ind),nU,nX), ...
        [], 1);
    
    descendant_matrix(k,:) = next_ind(fastsub2ind2([nU nX], J_min_idx, 1:nX));
    
    U_star_matrix(k,:) = J_min_idx;
    J = J_min;    
end

fprintf('\nComplete!\n');

% Store the results
dps.descendant_matrix = descendant_matrix;
dps.U_star_matrix     = U_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X1        = X1;
dps.X2        = X2;
dps.X3        = X3;
dps.U         = U;
dps.lb        = lb;
dps.ub        = ub;

dps.n_states  = 3;
dps.n_inputs  = 1;

dps.T_ocp     = T_ocp;
dps.T_dyn     = T_dyn;

dps.state_update_fn = state_update_fn;

end
