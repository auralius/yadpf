% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Time-optimal F8 aircraft
%
% Banks, S. P., & Mhana, K. J. (1992). Optimal control and stabilization 
% for nonlinear systems. IMA Journal of Mathematical Control and 
% Information, 9(2), 179–196. https://doi.org/10.1093/imamci/9.2.179
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
X1 = ( -0.2 : 0.01 : 0.5)';
X2 = ( -0.7 : 0.01 : 0.1)';
X3 = ( -0.9 : 0.01 : 0.9)';
U  = deg2rad(( -3: 0.5 : 3))';

% Setup the horizon
Ts = 0.1;            % Temporal discretization step
Tf = 10;             % From the paper, Tf < 7s
t = 0:Ts:Tf;
n_horizon = length(t);

% Initiate the solver
dps = dps_3X_1U(X1, X2, X3, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);

% Extract meaningful results for the given IC
dps = trace_3X_1U(dps, 0.452, 0, 0);

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
function J = stage_cost_fn(x1, x2, x3, u, ~)
global Ts;

% Tuning parameters q and r
q = 1;
r = 10;

J = Ts.*(q*(x1.^2) + q*(x2.^2) + q*(x3.^2) + r*u.^2);
end

%%
function J = terminal_cost_fn(~, x2, ~)
J = zeros(size(x2));
end