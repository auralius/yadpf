%% 
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Optimal storage strategy
%
% Jonsson, U. (2010). Optimal Control. 
% See Example 1, page 2
% https://www.math.kth.se/optsyst/grundutbildning/kurser/SF2852/LecturenotesScanned.pdf
%%
clear
close all
clc

global xf Ts;

% Setup the states and the inputs
X = 0 : 0.1 : 500;
U = 0 : 0.1 : 10;

% Setup the horizon
Ts = 1;           % Every 1 day, make sure to update Ts everywhere
T_vect = 0:Ts:30; % Day-0 to day-30
n_horizon = length(T_vect);

x0 = 0;           % Initial state
xf = 200;         % Terminal state

% Run the solver, trace forward, plot the results and the reachablility 
dps = dps_1X_1U(X, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);
dps = forward_trace(dps, x0);
plot_results(dps, '-d');
reachability_plot_1X(dps, 200, 10);

%% ------------------------------------------------------------------------
function [x_next] = state_update_fn(x, u)
global Ts;
x_next = Ts.*u + x; 
end

%% ------------------------------------------------------------------------
function J = stage_cost_fn(x, u, k)
global Ts;

r = 0.1; % production cost growth rate
c = 10;  % storage cost per time unit
J =  Ts*exp(r*k*Ts).*u + c*x*Ts;
end

%% ------------------------------------------------------------------------
function J = terminal_cost_fn(x)
global xf;
L = 100;
J = L .* (xf-x).^2;
end