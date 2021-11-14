% Auralius Manurung
% ME - Universitas Pertamina
% 2021

clear
close all
clc

% Setup the states and the inputs
X1 = 0 : 0.001 : 1; % Position
X2 = 0 : 0.001 : 1; % Velocity
U = -4 : 0.1   : 4; % Applied force

% Setup the horizon
Tf = 1;       % 1 second
T_ocp = 0.1;     % Temporal discretization step
t  = 0 : T_ocp : Tf;
n_horizon = length(t);

dpf.states           = {X1, X2};
dpf.inputs           = {U};
dpf.T_ocp            = T_ocp;
dpf.T_dyn            = 0.01;
dpf.n_horizon        = length(t);
dpf.state_update_fn  = @state_update_fn;
dpf.stage_cost_fn    = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [0 0]); % Initial state: [0 0]
yadpf_plot(dpf, '-');

% Reachability plot
yadpf_rplot(dpf, [0.5 0], 0.1);
%% ========================================================================
function X = state_update_fn(X, U, dt)
m = 1;   % Mass
b = 0.1; % Damping coefficient
 
X{1} = X{1} + dt*X{2};
X{2} = X{2} - b/m*dt.*X{2} + dt/m.*U{1};
end

%% ========================================================================
function J = stage_cost_fn(X, U, k, dt)
J = dt*U{1}.^2;
end

%% ========================================================================
function J = terminal_cost_fn(X)
xf = [0.5 0];

% Control gains
r1 = 100;
r2 = 1000;

J = r1*(X{1}-xf(1)).^2 + r2*(X{2}-xf(2)).^2;
end