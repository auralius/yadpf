%%
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Two-Tank Problem
%
% Elbert, P., Ebbesen, S., & Guzzella, L. (2013). Implementation of 
% dynamic programming for n-dimensional optimal control problems with final 
% state constraints. IEEE Transactions on Control Systems Technology, 
% 21(3), 924–931. https://doi.org/10.1109/TCST.2012.2190935

%%
clear
close all
clc

global Ts;

% Setup the states and the inputs
X1  = ( 0 : 0.01 : 1 )';
X2  = ( 0 : 0.01 : 1 )';
U1  = ( 0 : 0.5 : 1 )';
U2  = ( 0 : 0.5 : 1 )';

%  Set the horizon
Ts = 0.02;
t = 0:Ts:2;
n_horizon = length(t);

% Initiate the solver
dps = dps_2X_2U(X1, X2, U1, U2, n_horizon, @state_update_fn, ...
                @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results for a given initial condition
x1_ic = 0;
x2_ic = 0;
dps = trace_2X_2U(dps, x1_ic, x2_ic);

% Do plotting here
plot_2X_2U(dps, '-');


%%
function [x1_next, x2_next] = state_update_fn(x1, x2, u1, u2)
global Ts;

x1_next = Ts*(-0.5.*x1 + u1.*u2)     + x1;
x2_next = Ts*(-0.5.*x2 + u1.*(1-u2)) + x2;
end

%%
function J = stage_cost_fn(x1, x2, u1, u2, k)
global Ts;

J = (u1 + 1 .* abs(u2-0.5)) .* Ts;
end

%%
function J = terminal_cost_fn(x1, x2)


% Weighting factors
k1 = 100;
k2 = 100;

% Final states
x1f = 0.5;
x2f = 0.5;

J = k1.*(x1-x1f).^2 + k2.*(x2-x2f).^2;
end

