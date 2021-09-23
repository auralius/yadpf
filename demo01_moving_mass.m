clear
close all
clc

% Setup the states and the inputs
X = create_new_state('Position', 0, 1, 0.01);
V = create_new_state('Velocity', 0, 1, 0.01);
U = create_new_input('Force', -5, 5, 0.01);

states = [X V];

% Setup the horizon
Tf = 1;
dt = 0.1;
n_horizon = ceil(Tf/dt);

% Initiate the solver
dp = dps_2states_1input(states, U, n_horizon, @state_update_fn, @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results
[x1_star, x2_star, u_star] = trace_optimal_policy(dp.decendent_matrix, dp.U_star_matrix, X1, X2, nX1, nX2);

%%
function X_next = state_update_fn(X, U)
m = 1;
b = 0.1;
dt = 0.1;

x = X(1);
v = X(2);

x_next = x + dt * v;
v_next = v - b/m*v*dt + dt/m*U;

X_next = [x_next, v_next];
end

%%
function J = stage_cost_fn(X, U)
a1 = 1;
dt = 0.1;
J = a1*U^2*dt;
end

%%
function J = terminal_cost_fn(X)
% Weighting factors
a2 = 1000;
a3 = 1000;

% Final states
xf = 0.7;
vf = 0;

J = a2*(X(1)-xf)^2 + a3*(X(2)-vf)^2;
end