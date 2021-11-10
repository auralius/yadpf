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
%     T_ocp             = sampling time for the optimal control problem
%     T_dyn             = sampling time for the system's dynamic simulation
%
function dps = dps_2X_1U(X1, ...
    X2, ...
    U, ...
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

% Frequently used parameters/variables
nU    = length(U);
nX1   = length(X1);
nX2   = length(X2);
nX    = nX1*nX2;
lb    = [min(X1) min(X2)];
ub    = [max(X1) max(X2)];

% Where to keep the results?
U_star_matrix     = zeros(n_horizon, nX, 'uint32');      % Store the optimal inputs
descendant_matrix = zeros(n_horizon, nX, 'uint32');      % Store the optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Calculating terminal cost...\n')

% The terminal cost is only a function of the state variables
[r, c] = ind2sub([nX1 nX2], 1:nX);
J = terminal_cost_fn(X1(r), X2(c));

i = fastreprowvec(1:nX, nU);
[r, c] = ind2sub([nX1 nX2], i);

n_iter = T_ocp/T_dyn; 
x1_next = X1(r);
x2_next = X2(c);
UU = fastrepcolvec(U',nX);

% Simulate the system with sampling period of Tdyn
fprintf('Simulating the system one step ahead...\n');
fprintf('Progress: ');

ll = 0;
for k = 1 : 1 : n_iter
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',k/n_iter*100);
    [x1_next, x2_next] = state_update_fn(x1_next, x2_next, UU, T_dyn);
end

clear UU;

% Bound the states within the minimum and maximum values
x1_next = min(max(x1_next, lb(1)), ub(1));
x2_next = min(max(x2_next, lb(2)), ub(2));

r = snap(x1_next, repmat(lb(1),nU,nX), repmat(ub(1),nU,nX), ...
    repmat(nX1,nU,nX));
c = snap(x2_next, repmat(lb(2),nU,nX), repmat(ub(2),nU,nX), ...
    repmat(nX2,nU,nX));

next_ind = fastsub2ind2([nX1 nX2], r, c);

fprintf('\nComplete!\n');

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = n_horizon-1 : -1 : 1
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);
    
    J_old = J;
    [J_min, J_min_idx] = min(stage_cost_fn(X1(r), X2(c), ...
                             fastrepcolvec(U',nX), k, T_ocp) + ...
                             reshape(J_old(next_ind),nU,nX), [], 1);
    
    descendant_matrix(k,:) = next_ind(fastsub2ind2([nU nX], J_min_idx, 1:nX));
    
    U_star_matrix(k,:) = J_min_idx;
    J = J_min;
end

fprintf('\nComplete!\n');

% Store the results
dps.J                 = J;
dps.descendant_matrix = descendant_matrix;
dps.U_star_matrix     = U_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X1        = X1;
dps.X2        = X2;
dps.U         = U;
dps.lb        = lb;
dps.ub        = ub;

dps.nX1       = nX1;
dps.nX2       = nX2;
dps.nX        = nX;
dps.nU        = nU;

dps.n_states  = 2;
dps.n_inputs  = 1;

dps.T_ocp     = T_ocp;
dps.T_dyn     = T_dyn;

dps.state_update_fn = state_update_fn;

end
