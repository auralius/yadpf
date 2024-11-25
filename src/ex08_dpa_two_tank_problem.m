%%
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Two-Tank Problem
%
% Elbert, P., Ebbesen, S., & Guzzella, L. (2013). Implementation of 
% dynamic programming for n-dimensional optimal control problems with final 
% state constraints. IEEE Transactions on Control Systems Technology, 
% 21(3), 924–931. https://doi.org/10.1109/TCST.2012.2190935

%%
clear
close all
clc

% Setup the states and the inputs
X1  = 0 : 0.001 : 1;
X2  = 0 : 0.001 : 1;
U1  = 0 : 0.1   : 1;
U2  = 0 : 0.1   : 1 ;

%  Set the horizon
T_ocp = 0.2;
t    = 0 : T_ocp : 2;
n_horizon = length(t);

% Initiate the solver
dpf.states = {X1 X2};
dpf.inputs = {U1 U2};
dpf.T_ocp = T_ocp;
dpf.T_dyn = 0.01;
dpf.n_horizon = length(t);
dpf.state_update_fn = @state_update_fn;
dpf.stage_cost_fn = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [0 0]);
yadpf_plot(dpf, '-');

% Reachability plot, this takes time and the plot will not be very
% responsive!
yadpf_rplot(dpf, [0.5 0.5], 0.01); 

%%
function X = state_update_fn(X, U, dt)
X{1} = dt*(-0.5*X{1} + U{1}.*U{2})       + X{1};
X{2} = dt*(-0.5*X{2} + U{1}.*(1-U{2})) + X{2};
end

%%
function J = stage_cost_fn(X, U, k, dt)
J = (U{1} + 0.1 .* abs(U{2}-0.5)) .* dt;
end

%%
function J = terminal_cost_fn(X)
% Final states
xf = [0.5 0.5];
e1 = X{1}-xf(1);
e2 = X{2}-xf(2);

J1 = zeros(size(X{1}));
J2 = zeros(size(X{2}));

J1(e1<0) = 1000;
J2(e2<0) = 1000;
J = (J1 + J2);
end

