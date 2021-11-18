%%
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Two-Tank Problem
%

%%
clear
close all
clc

% Setup the states and the inputs
X  = 0 : 0.001 : 1;
U1  = [0 1];
U2  = [0 0.5];

%  Set the horizon
T_ocp = 0.1;
t = 0 : T_ocp : 2;
n_horizon = length(t);

% Initiate the solver
dpf.states = {X};
dpf.inputs = {U1 U2};
dpf.T_ocp = T_ocp;
dpf.T_dyn = T_ocp;
dpf.n_horizon = length(t);
dpf.state_update_fn = @state_update_fn;
dpf.stage_cost_fn = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, 0);
yadpf_plot(dpf, '-');

%%
function X = state_update_fn(X, U, dt)
X{1} = dt*(-0.5.*X{1} + U{1} + U{2}) + X{1};
end

%%
function J = stage_cost_fn(X, U, k, dt)
xf = 1;
J = (U{1}.^2 + U{2}.^2)*dt + 10*dt*(X{1}-xf).^2;
end

%%
function J = terminal_cost_fn(X)
xf = 1;
J = 100*(X{1}-xf).^2;
end

