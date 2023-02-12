% Auralius Manurung (manurung.auralius@gmail.com)
% ME - Universitas Pertamina
%
% January 2022
%
% Teo, K. L., Lee, H. W. J., & Rehbock, V. (1998). Control parametrization
% enhancing technique for time optimal control and optimal three-valued
% control problems. Dynamics of Continuous, Discrete and Impulsive Systems
% Series B: Application and Algorithm, 4(4), 617–631.
%
% See page 18 Section 5.3
%

clear
close all
clc

%%
function X = state_update_fn(X, U, dt)
X{1} = dt*((1 - X{1}).*U{1} + (2-X{1}).*U{2}) ./ X{2} + X{1};
X{2} = dt*(-0.02*sqrt(X{2}) + U{1} + U{2}) + X{2};
end

%%
function J = stage_cost_fn(X, U, ~, dt)
% Tuning parameters q and r
q1  = 0.25;
q2  = 0.25;
r1  = 1;
r2  = 1;

xf = [1.25 1];

J = dt.*(q1*(X{1}-xf(1)).^2 + q2*(X{2}-xf(2)).^2 ...
    + r1*U{1}.^2 + r2*U{2}.^2 );
end

% ------------------------------------------------------------------------------
% Setup the grids for the states and the inputs
X1 = 0.6 : 0.001  : 1.3;
X2 = 0.6 : 0.001  : 1.3;
U1 = [0 0.03];
U2 = [0 0.01];

% Initial and terminal states
x0 = [0.8 0.7];
xf = [1.25 1.0];

% Initiate the solver
dpf.states              = {X1 X2};
dpf.inputs              = {U1 U2};
dpf.T_ocp               = 1;
dpf.T_dyn               = 0.01;
dpf.max_iter            = 200;
dpf.state_update_fn     = @state_update_fn;
dpf.stage_cost_fn       = @stage_cost_fn;

% Initiate and run the solver, do forward tracing and plot the results
dpf = yadpf_visolve(dpf);
dpf = yadpf_vitrace(dpf, x0, xf, 50);
yadpf_plot(dpf, '-');


