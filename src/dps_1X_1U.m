% Dynamic programming solver for a system with one input variable and one 
% state variable
%
% function dps = dps_1X_1U(X, ...
%                U, ...
%                n_horizon, ...
%                state_update_fn, ...
%                stage_cost_fn, ...
%                terminal_cost_fn, ..
%                Tocp, ...
%                Tdyn)
%
% Arguments:
%
%     X                 = discretized state (nX x 1)
%     U                 = discretized input (nU x 1)
%     n_horizon         = horizon length (a positive integer)
%     state_update_fn   = user defined system model
%     stage_cost_fn     = user defined stage cost function
%     terminal_cost_fn  = user defined terminal cost function
%    Tocp              = sampling time for the optimal control problem
%    Tdyn              = sampling time for the system's dynamic simulation
%
function dps = dps_1X_1U(X, ...
    U, ...
    n_horizon, ...
    state_update_fn, ...
    stage_cost_fn, ...
    terminal_cost_fn, ...
    T_ocp, ...
    T_dyn)

% Check the arguments
if nargin < 7
    T_ocp = 1;
    T_dyn = T_ocp;
elseif nargin < 8
    T_dyn = T_ocp;
end 

if T_dyn > T_ocp
    error('Time step for simulating the system must be less or equal than the time step for the OCP discretization!')
end

nU = length(U);
nX = length(X);
lb = min(X);
ub = max(X);

U_star_matrix     = zeros(n_horizon, nX, 'uint32');      % Store the optimal inputs
descendant_matrix = zeros(n_horizon, nX, 'uint32');      % Store the optimal next state

fprintf('Horizons : %i stages\n',n_horizon);
fprintf('State    : %i nodes\n', nX);
fprintf('Input    : %i nodes\n', nU);
fprintf('Calculating terminal cost...\n');

% The terminal cost is only a function of the state variables
J = terminal_cost_fn(X);

% Simulate the system with sampling period of Tdyn
fprintf('Simulating the system one step ahead...\n');
fprintf('Progress: ');

n_iter = T_ocp/T_dyn; 

i = fastreprowvec(1:nX, nU);
x_next = X(i);
UU = fastrepcolvec(U',nX);

ll = 0;
for k = 1 : 1 : n_iter
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',k/n_iter*100);
    x_next = state_update_fn(x_next, UU, T_dyn);
end

clear UU;

% Bound the states within the minimum and maximum values
x_next = min(max(x_next, lb), ub);

ind = snap(x_next, ...
    fastsca2mat(lb,nU,nX), fastsca2mat(ub,nU,nX), fastsca2mat(nX,nU,nX));

fprintf('\nCompleted!\n');

% Stage-wise iteration
fprintf('Running backward dynamic programming algorithm...\n');
fprintf('Stage-');
ll = 0;

for k = n_horizon-1 : -1 : 1
    J_old = J;
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%i',k);
       
    [J_min, J_min_idx] = min(stage_cost_fn(X(i), fastrepcolvec(U',nX), k, T_ocp) ...
                             + reshape(J_old(ind),nU,nX), [], 1); 
    
    descendant_matrix(k,:) = ind(fastsub2ind2([nU nX], J_min_idx, 1:nX));
        
    U_star_matrix(k,:) = J_min_idx;
    J = J_min;
end

fprintf('\nCompleted!\n');

% Store the results
dps.descendant_matrix = descendant_matrix;
dps.U_star_matrix = U_star_matrix;

% Additional information that might be still needed
dps.n_horizon = n_horizon;
dps.X         = X;
dps.U         = U;
dps.nX        = nX;
dps.nU        = nU;
dps.lb        = lb;
dps.ub        = ub;

dps.n_states  = 1;
dps.n_inputs  = 1;

dps.T_ocp = T_ocp;
dps.T_dyn = T_dyn;

dps.state_update_fn = state_update_fn;

end
