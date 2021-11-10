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

global x0 xf;

% Setup the states and the inputs
X1 = -1   : 0.01 : 2;
X2 = -1   : 0.01 : 1;
U  = [-1 0 1];

% Setup the horizon
T_ocp = 0.1;         % Temporal discretization step
Tf = 4;              % Final time
t = 0 : T_ocp : Tf;
n_horizon = length(t);

x0 = [1 1];          % Initial states
xf = [0 0];          % Terminal states

% Initiate and run the solver, do forward tracing and plot the results
dps = dps_2X_1U(X1, X2, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn, T_ocp, 0.1);
dps = forward_trace(dps, x0);
plot_results(dps, '-');

% Additional plotting
figure
plot(dps.x1_star, dps.x2_star, '-', 'LineWidth', 2);
xlabel('$x_1$', 'Interpreter','latex')
ylabel('$x_2$', 'Interpreter','latex')
axis equal


%%
function [x1_next, x2_next] = state_update_fn(x1, x2, u, dt)
if (u == 0)
    x1_next = x1;
    x2_next = x2;
else
    x1_next = dt*x2 + x1;
    x2_next = dt*(-x1 - (x1.^2-1) .* x2 + u) + x2;
end
end

%%
function J = stage_cost_fn(x1, x2, u, k, dt)
global xf;

q = 100;
r = 1;
J = dt * (q*(x1-xf(1)).^2 + q*(x2-xf(2)).^2 + r*u.^2);
end

%%
function J = terminal_cost_fn(x1, x2)
global xf;

q = 100;
J = q*(x1-xf(1)).^2 + q*(x2-xf(2)).^2;
end