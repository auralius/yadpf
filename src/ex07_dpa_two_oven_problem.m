% Auralius Manurung
% ME - Universitas Pertamina
% 2021
% 
% Two-Oven Problem
%
% https://www.mit.edu/~dimitrib/DP_Slides_2015.pdf
% See page 22: LINEAR-QUADRATIC ANALYTICAL EXAMPLE
%
%%
clear
close all
clc

% Initial and terminal states
x0 = 30;
xf = 300;

% Setup the states and the inputs
X = 0 : 0.1 : 1000;
U = 0 : 0.1 : 1000;

% Build the structure
dpf.states              = {X};
dpf.inputs              = {U};
dpf.T_ocp               = 1;
dpf.T_dyn               = 1;
dpf.n_horizon           = 3;
dpf.state_update_fn     = @state_update_fn;
dpf.stage_cost_fn       = @stage_cost_fn;
dpf.terminal_cost_fn    = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, x0);
yadpf_plot(dpf, '-');

% Reachability plot
yadpf_rplot(dpf, xf, 10);

%% The state update function
function X = state_update_fn(X, U, ~)
a = 0.7;
X{1} = (1 - a).*X{1} + a * U{1};
end

%% The stage cost function
function J = stage_cost_fn(X, U, k, ~)
J = U{1}.^2;
end

%% The terminal cost function
function J = terminal_cost_fn(X)
xf = 300; % Desired terminal state

% Weighting factor
r = 1000;

J = r * (X{1}-xf).^2;
end