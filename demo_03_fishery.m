%% 
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Lotka-Volterra Fishery Problem

%%
clear
close all
clc

% Setup the states and the inputs
X = ( 0 : 1   : 1000)';
U = ( 0 : 0.1 : 10 )';

% Setup the horizon
Tf = 200;   % 200 days
Ts = 0.2;   % Every 0.2 days
T_vect = 0:Ts:Tf;
n_horizon = length(T_vect);

% Initiate the solver
dps = dps_1X_1U(X, U, n_horizon, @state_update_fn, @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results for a given initial condition
x0 = 250;
dps = trace_1X_1U(dps, x0);

% Do plotting here
plot_1X_1U(dps);

%%
function [x_next] = state_update_fn(x, u)
Ts = 0.2;
f = Ts .* (0.02 .* (x-x.^2./ 1000) - u);
x_next = f + x; 
end

%%
function J = stage_cost_fn(x, u,k)
Ts = 0.2;
J =  -Ts .* u;
end

%%
function J = terminal_cost_fn(x)
r = 100;
J = r .* (750-x).^2;
end