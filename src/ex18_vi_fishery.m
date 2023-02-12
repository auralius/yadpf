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

%% The state update function
function X = state_update_fn(X, U, dt)
X{1} = X{1} + dt * (0.02 * (X{1}-X{1}.^2./ 1000) - U{1});
end

%% The stage cost function
function J = stage_cost_fn(X, U, k, dt)
xf = 750; % Terminal state

r1 = 100;
r2 = 0.5;

J =  dt *(-r1*U{1} + r2*(xf-X{1}).^2);
end

% ------------------------------------------------------------------------------
% Initial and terminal state
x0 = 250;
xf = 750;

% Setup the states and the inputs
X = 0 : 0.1 : 1000;
U = 0 : 1   : 10;

dpf.states           = {X};
dpf.inputs           = {U};
dpf.T_ocp            = 1;
dpf.T_dyn            = 0.01;
dpf.max_iter         = 200;
dpf.state_update_fn  = @state_update_fn;
dpf.stage_cost_fn    = @stage_cost_fn;

% Initiate and run the solver, do forward tracing and plot the results
dpf = yadpf_visolve(dpf, 0.95);
dpf = yadpf_vitrace(dpf, x0, xf,200);
yadpf_plot(dpf, '-');

% Policy plot
yadpf_pplot(dpf)


