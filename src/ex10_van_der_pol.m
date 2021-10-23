% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% The Time-optimal van der Pol equation with a conbtrol input
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
X1 = -3   : 0.01 : 2;
X2 = -4   : 0.01 : 2;
U  = [-1 0 1];

% Setup the horizon
Ts = 0.1;            % Temporal discretization step
Tf = 5;              % 
t = 0:Ts:Tf;
n_horizon = length(t);

% Initiate the solver
dps = dps_2X_1U(X1, X2, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);

% Extract meaningful results
dps = firward_trace(dps, [1.0 1.0]);

% Do plotting here
plot_results(dps, '-');

% Additional plotting
figure
plot(dps.x1_star, dps.x2_star, '-o', 'LineWidth', 2);
xlabel('$x_1$', 'Interpreter','latex')
ylabel('$x_2$', 'Interpreter','latex')
axis equal


%%
function [x1_next, x2_next] = state_update_fn(x1, x2, u)
global Ts;

x1_next = Ts.*x2 + x1;
x2_next = Ts.*(-x1 - (x1.^2-1) .* x2 + u) + x2;
end

%%
function J = stage_cost_fn(x1, x2, u, k)
global Ts;
x1T = 0;
x2T = 0;

r1 = 1000;
r2 = 1000;

J = Ts .* (r1.*(x1-x1T).^2 + r2.*(x2-x2T).^2);
end

%%
function J = terminal_cost_fn(x1, x2)
J = zeros(size(x1));
end