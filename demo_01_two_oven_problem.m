% Auralius Manurung
% ME - Universitas Pertamina
% 2021
% 
% Two-Oven Problem
%
% https://www.mit.edu/~dimitrib/DP_Slides_2015.pdf
% See page 22: LINEAR-QUADRATIC ANALYTICAL EXAMPLE

clear
close all
clc

% Setup the states and the inputs
X = (0:1:1000)';
U = (0:1:1000)';

% Setup the horizon
n_horizon = 3;

% Initiate the solver
dps = dps_1X_1U(X, U, n_horizon, @state_update_fn, @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results
x0 = 30;
dps = trace_1X_1U(dps, x0);

% Plot the results
plot_1X_1U(dps, '-');

%%
function x_next = state_update_fn(x, u)
a = 0.7;
x_next = (1 - a).*x + a .* u;
end

%%
function J = stage_cost_fn(x, u, k)
J = u.^2;
end

%%
function J = terminal_cost_fn(x)
% Desired terminal state value
T = 300;

% Weighting factor
r = 100;

J = r * (x-T).^2;
end