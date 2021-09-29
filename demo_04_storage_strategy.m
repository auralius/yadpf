%% 
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Optimal storage strategy

%%
clear
close all
clc

% Setup the states and the inputs
X = ( 0 : 0.1 : 500)';
U = ( 0 : 0.1 : 10 )';

% Setup the horizon
Tf = 30;   % 200 days
Ts = 1;    % Every 1 day
T_vect = 0:Ts:Tf;
n_horizon = length(T_vect);

% Initiate the solver
dps = dps_1X_1U(X, U, n_horizon, @state_update_fn, @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results for a given initial condition
x0 = 0;
dps = trace_1X_1U(dps, x0);

% Do plotting here
plot_1X_1U(dps);

%%
function [x_next] = state_update_fn(x, u)
Ts = 1;
x_next = Ts.*u + x; 
end

%%
function J = stage_cost_fn(x, u, k)
Ts = 1;  % Step size
r = 0.1; % production cost growth rate
c = 10;  % storage cost per time unit
J =  Ts*exp(r*k*Ts).*u + c*x*Ts;
end

%%
function J = terminal_cost_fn(x)
r = 1000;
xf = 200;
Ts = 1;
J = Ts*r .* (xf-x).^2;
end