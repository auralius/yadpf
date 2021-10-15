% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Time-optimal F8 aircraft
%
% Kaya, C. Y., & Noakes, J. L. (2003). Computational Method for 
% Time-Optimal Switching Control. Journal of Optimization Theory and 
% Applications, 117(1), 69–92. https://doi.org/10.1023/A:1023600422807
%

clear
close all
clc

global Ts;

% Setup the states and the inputs
% In degrees, our state resolutions are about 0.6 deg
X1 = single(( -0.9 : 0.01 : 0.9)');
X2 = single(( -1.5 : 0.01 : 0.5)');
X3 = single(( -0.9 : 0.01 : 0.9)');
U  = single([-0.05236 0 0.05236]');

% Setup the horizon
Ts = 0.1;            % Temporal discretization step
Tf = 7;              % From the paper, Tf < 7s
t = 0:Ts:Tf;
n_horizon = length(t);

% Initiate the solver
dps = dps_3X_1U(X1, X2, X3, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);

% Extract meaningful results for the given IC
dps = trace_3X_1U(dps, 0.4655, 0, 0);

% Do plotting here
plot_3X_1U(dps, '-');

%%
function [x1_next, x2_next, x3_next] = state_update_fn(x1, x2, x3, u)
global Ts;

x1_next = (-0.877*x1 + x3 - 0.088*(x1.*x3) + 0.47.*(x1.^2) - ...
          0.019*(x2.^2) - (x1.^2).*x3 + 3.846*(x1.^3) - 0.215*u + ...
          0.28*((x1.^2).*u) + 0.47*(x1.*(u.^2)) + 0.63*(u.^3))*Ts + x1;
x2_next = x3*Ts + x2;
x3_next = (-4.208*x1 - 0.396*x3 - 0.47*(x1.^2) - 3.564*(x1.^3) - ...
          20.967*u + 6.265*((x1.^2).*u) + 46*(x1.*(u.^2)) + ...
          61.4*u.^3)*Ts + x3;
end

%%
function J = stage_cost_fn(x1, x2, x3, u, k)
global Ts;

% k1, k2, k3, k4 are the tuning parameters
k1 = 10;
k2 = 10;
k3 = 0.01;
k4 = 100;

J = Ts.*(k1*(x1.^2) + k2*(x2.^2) + k3*(x3.^2) + k4*u.^2);
end

%%
function J = terminal_cost_fn(x1, x2, x3)
J = zeros(size(x2));
end