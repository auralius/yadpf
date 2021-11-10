%%
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Two-Tank Problem
%

%%
clear
close all
clc

% Setup the states and the inputs
X  = 0 : 0.001 : 1;
U1  = [0 1];
U2  = [0 0.5];

%  Set the horizon
T_ocp = 0.1;
t = 0 : T_ocp : 2;
n_horizon = length(t);

% Initiate the solver
dps = dps_1X_2U(X,U1, U2, n_horizon, @state_update_fn, ...
                @stage_cost_fn, @terminal_cost_fn, T_ocp);

% Extract meaningful results for a given initial condition
x_ic = 0;
dps = forward_trace(dps, x_ic);

% Do plotting here
plot_results(dps, '-');


%%
function [x_next] = state_update_fn(x, u1, u2, dt)
x_next = dt*(-0.5.*x + u1 + u2) + x;
end

%%
function J = stage_cost_fn(x, u1, u2, k, dt)
xf = 1;
J = (u1.^2 + u2.^2)*dt + 10*dt*(x-xf).^2;
end

%%
function J = terminal_cost_fn(x)
xf = 1;
J = 100*(x-xf).^2;
end

