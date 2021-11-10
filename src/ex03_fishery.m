%% 
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Lotka-Volterra Fishery Problem
%
% Sundstrom, O., & Guzzella, L. (2009). A Generic Dynamic Programming 
% Matlab Function. 18th IEEE International Conference on Control 
% Applications, 7, 1625–1630. https://doi.org/10.1109/CCA.2009.5281131

%%
clear
close all
clc

global xf;

% Initial and terminal state
x0 = 250;
xf = 750;

% Setup the states and the inputs
X = 0 : 0.1 : 1000;
U = 0 : 1   : 10;

% Setup the horizon
Tf = 200;     % 200 days
T_ocp = 0.2;  % Every half day
T_vect = 0 : T_ocp : Tf;
n_horizon = length(T_vect);

% Initiate and run the solver, pdo forward tracing for the desirec IC.
% plot the results
dps = dps_1X_1U(X, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn, T_ocp);
dps = forward_trace(dps, x0);
plot_results(dps, '-');
reachability_plot_1X(dps, xf, 5);

%% ------------------------------------------------------------------------
function [x_next] = state_update_fn(x, u, dt)
f = dt .* (0.02 .* (x-x.^2./ 1000) - u);
x_next = f + x; 
end

%% ------------------------------------------------------------------------
function J = stage_cost_fn(x, u, k, dt)
J =  -dt .* u;
end

%% ------------------------------------------------------------------------
function J = terminal_cost_fn(x)
global xf;
r = 1000;
J = r .* (xf-x).^2;
end