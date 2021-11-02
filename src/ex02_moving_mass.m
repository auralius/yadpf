% Auralius Manurung
% ME - Universitas Pertamina
% 2021

clear
close all
clc

global dt x0 xf;

% Initial and terminal states: [Pos Vel]
x0 = [0 0];
xf = [0.5 0];

% Setup the states and the inputs
X1 = 0 : 0.001 : 1; % Position
X2 = 0 : 0.001 : 1; % Velocity
U = -5 : 0.1   : 5; % Applied force

% Setup the horizon
Tf = 1;       % 1 second
dt = 0.1;     % Temporal discretization step
t  = 0:dt:Tf;
n_horizon = length(t);

% Initiate and run the solver, do forwar tracing and plot the results
dps = dps_2X_1U(X1, X2, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);
dps = forward_trace(dps, x0);
plot_results(dps, '-');
reachability_plot_2X(dps, [0.5 0], 0.1);

%% ========================================================================
function [x1_next, x2_next] = state_update_fn(x1, x2, u)
global dt;

m = 1;   % Mass
b = 0.1; % Damping coefficient
 
x1_next = x1 + dt*x2;
x2_next = x2 - b/m*dt.*x2 + dt/m.*u;
end

%% ========================================================================
function J = stage_cost_fn(x1, x2, u, k)
global dt;
J = dt*u.^2;
end

%% ========================================================================
function J = terminal_cost_fn(x1, x2)
global xf;

% Control gains
r1 = 100;
r2 = 1000;

J = r1.*(x1-xf(1)).^2 + r2.*(x2-xf(2)).^2;
end