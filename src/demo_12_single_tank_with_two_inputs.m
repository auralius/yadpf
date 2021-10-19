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

global Ts;

% Setup the states and the inputs
X  = ( 0 : 0.001 : 1 )';
U1  = [0 1]';
U2  = [0 0.5]';

%  Set the horizon
Ts = 0.1;
t = 0:Ts:2;
n_horizon = length(t);

% Initiate the solver
dps = dps_1X_2U(X,U1, U2, n_horizon, @state_update_fn, ...
                @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results for a given initial condition
x_ic = 0;
dps = trace_1X_2U(dps, x_ic);

% Do plotting here
plot_1X_2U(dps, '-');


%%
function [x_next] = state_update_fn(x, u1, u2)
global Ts;

x_next = Ts*(-0.5.*x + u1 + u2) + x;
end

%%
function J = stage_cost_fn(x, u1, u2, k)
global Ts;
xf = 1;
J = (u1.^2 + u2.^2)* Ts + 10*Ts*(x-xf).^2;
end

%%
function J = terminal_cost_fn(x)
xf = 1;
J = 100*(x-xf).^2;
end

