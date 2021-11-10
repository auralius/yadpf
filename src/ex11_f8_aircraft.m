% Auralius Manurung (manurung.auralius@gmail.com)
% ME - Universitas Pertamina
% November 2021
%
% Stabilization of an F8 aircraft
%
% Banks, S. P., & Mhana, K. J. (1992). Optimal control and stabilization 
% for nonlinear systems. IMA Journal of Mathematical Control and 
% Information, 9(2), 179–196. https://doi.org/10.1093/imamci/9.2.179
%
% Kaya, C. Y., & Noakes, J. L. (2003). Computational Method for 
% Time-Optimal Switching Control. Journal of Optimization Theory and 
% Applications, 117(1), 69–92. https://doi.org/10.1023/A:1023600422807
%
% Garrard, W. L., & Jordan, J. M. (1977). Design of nonlinear automatic 
% flight control systems. Automatica, 13(5), 497–505. 
% https://doi.org/10.1016/0005-1098(77)90070-X
%
% Kaya, C. Y., & Noakes, J. L. (1996). Computations and time-optimal 
% controls. Optimal Control Applications and Methods, 17(3), 171–185. 
% https://doi.org/10.1002/(SICI)1099-1514(199607/09)17:3<171::AID-OCA571>3.0.CO;2-9
%
% This may require at leasr 16GB of RAM!
%

clear
close all
clc

% Setup the grids for the states and the inputs
X1 = -0.2 : 0.01  : 0.6;   % Attack angle : alpha
X2 = -0.6 : 0.01  : 0.3;   % Pitch angle : theta
X3 = -1.2 : 0.01  : 0.7;   % Pitch rate : theta_dot
U  = deg2rad([-3 0 3]);    % Tail deflection

% Setup the horizon
Topt = 0.2;  % Discretization time step, less than 0.3s requires finer grid
Tf = 10;     % From the paper, Tf < 7s, 10s should be fine
t = 0 : Topt : Tf;
n_horizon = length(t);

% Initiate and run the solver
% Dynamics simulation runs at 100 Hz, this can take a some time!
dps = dps_3X_1U(X1, X2, X3, U, n_horizon, @state_update_fn, ...
                @stage_cost_fn, @terminal_cost_fn, Topt, 0.01);

% Extract meaningful results for the given IC
dps = forward_trace(dps, [deg2rad(26.7) 0 0]);

% Do plotting here
plot_results(dps, '-');

%%
function [x1_next, x2_next, x3_next] = state_update_fn(x1, x2, x3, u, dt)
x1_next = (-0.877*x1 + x3 - 0.088*(x1.*x3) + 0.47.*(x1.^2) - ...
          0.019*(x2.^2) - (x1.^2).*x3 + 3.846*(x1.^3) - 0.215*u + ...
          0.28*((x1.^2).*u) + 0.47*(x1.*(u.^2)) + 0.63*(u.^3))*dt + x1;
x2_next = x3*dt + x2;
x3_next = (-4.208*x1 - 0.396*x3 - 0.47*(x1.^2) - 3.564*(x1.^3) - ...
          20.967*u + 6.265*((x1.^2).*u) + 46*(x1.*(u.^2)) + ...
          61.4*u.^3)*dt + x3;
end

%%
function J = stage_cost_fn(x1, x2, x3, u, ~, dt)
% Tuning parameters q and r
q1 = 100; % attack angle
q2 = 100; % pitch
q3 = 100; % pitch rate
r  = 0;   % input can be zero, so we apply input minimizer, but not much

J = dt.*(q1*(x1.^2) + q2*(x2.^2) + q3*(x3.^2) + r*u.^2);
end

%%
function J = terminal_cost_fn(x1, x2, x3)
J = 100*((x1.^2) + (x2.^2) + (x3.^2) );
end