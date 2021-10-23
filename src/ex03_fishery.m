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

global Ts xf;

% Initial and terminal state
x0 = 250;
xf = 750;

% Setup the states and the inputs
X = 0 : 0.1 : 1000;
U = 0 : 1   : 10;

% Setup the horizon
Tf = 200;   % 200 days
Ts = 0.5;   % Every 1 day
T_vect = 0:Ts:Tf;
n_horizon = length(T_vect);

% Initiate the solver
dps = dps_1X_1U(X, U, n_horizon, @state_update_fn, @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results for a given initial condition
dps = forward_trace(dps, x0);

% Do plotting here
plot_results(dps, '-');

% Reachability plot here
reachability_plot_1X(dps, xf, 5);
%%
function [x_next] = state_update_fn(x, u)
global Ts;
f = Ts .* (0.02 .* (x-x.^2./ 1000) - u);
x_next = f + x; 
end

%%
function J = stage_cost_fn(x, u,k)
global Ts;
J =  -Ts .* u;
end

%%
function J = terminal_cost_fn(x)
global xf;
r = 1000;
J = r .* (xf-x).^2;
end