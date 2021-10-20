% Auralius Manurung
% ME - Universitas Pertamina
% 2021

clear
close all
clc

global dt;

% Setup the states and the inputs
X = 0  : 0.001 : 1;
V = 0  : 0.001 : 1;
F = -5 : 0.1   : 5;

% Setup the horizon
Tf = 1;     % 1 second
dt = 0.1;   % Temporal discretization step
t = 0:dt:Tf;
n_horizon = length(t);

% Initiate the solver
dps = dps_2X_1U(X, V, F, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);

% Extract meaningful results
dps = trace_2X_1U(dps, 0, 0);

% Do plotting here
plot_2X_1U(dps, '-');

%%
function [x1_next, x2_next] = state_update_fn(x1, x2, u)
global dt;

m = 1;
b = 0.1;
 
x1_next = x1 + dt*x2;
x2_next = x2 - b/m*dt.*x2 + dt/m.*u;

end

%%
function J = stage_cost_fn(x1, x2, u, k)
global dt;

a1 = 1;
J = a1*dt*u.^2;
end

%%
function J = terminal_cost_fn(x1, x2)
% Weighting factors
a2 = 100;
a3 = 100;

% Final states
xf = 0.5;
vf = 0;

J = a2.*(x1-xf).^2 + a3.*(x2-vf).^2;
end