%% 
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Lotka-Volterra Fishery Problem
%
% Sundstrom, O., & Guzzella, L. (2009). A Generic Dynamic Programming 
% Matlab Function. 18th IEEE International Conference on Control 
% Applications, 7, 1625–1630. https://doi.org/10.1109/CCA.2009.5281131

%%
clear
close all
clc

% Initial and terminal state
x0 = 250;
xf = 750;

% Setup the states and the inputs
X = 0 : 0.1 : 1000;
U = 0 : 1   : 10;

% Setup the horizon
Tf = 200;     % 200 days
T_ocp = 0.2;  % Every half day
t = 0 : T_ocp : Tf;

dpf.states           = {X};
dpf.inputs           = {U};
dpf.T_ocp            = T_ocp;
dpf.T_dyn            = 0.01;
dpf.n_horizon        = length(t);
dpf.state_update_fn  = @state_update_fn;
dpf.stage_cost_fn    = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forward tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, x0);
yadpf_plot(dpf, '-');

% Draw the reachability plot
yadpf_rplot(dpf, xf, 5);

%% The state update function
function X = state_update_fn(X, U, dt)
X{1} = X{1} + dt * (0.02 * (X{1}-X{1}.^2./ 1000) - U{1});
end

%% The stage cost function
function J = stage_cost_fn(X, U, k, dt)
J =  -dt .* U{1};
end

%% The terminal cost function
function J = terminal_cost_fn(X)
xf = 750; % Terminal state
r = 1000;
J = r * (xf-X{1}).^2;
end