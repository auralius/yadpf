% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% A mass (1 kg) is moving from x=0 to x=0.5 in exactly 1 second because of
% an exteral force. The damping coefficient is 0.1. At the destination, the
% mass must stop moving. The external force is bounded (-4 N to 4 N).
%
% The objective function is: minimum integral of squared error (ISE)
%

clear
close all
clc

%% ========================================================================
function X = state_update_fn(X, U, dt)
m = 1;   % Mass
b = 0.1; % Damping coefficient

X{1} = X{1} + dt*X{2};
X{2} = X{2} - b/m*dt.*X{2} + dt/m.*U{1};
end

%% ========================================================================
function J = stage_cost_fn(X, U, k, dt)
xf = [0.5 0]; % Terminal states

% Control gains
r1 = 1000;
r2 = 10;

J = dt*(r1*(X{1}-xf(1)).^2 + r2*(X{2}-xf(2)).^2);
end

% ========================================================================
% Initial and terminal states: [Pos Vel]
x0 = [0 0];
xf = [0.5 0];

% Setup the states and the inputs
X1 = 0 : 0.001 : 1; % Position
X2 = 0 : 0.001 : 1; % Velocity
U = -4 : 0.1   : 4; % Applied force

dpf.states           = {X1 X2};
dpf.inputs           = {U};
dpf.T_ocp            = 0.1;
dpf.T_dyn            = 0.01;
dpf.max_iter         = 200;
dpf.state_update_fn  = @state_update_fn;
dpf.stage_cost_fn    = @stage_cost_fn;

% Initiate and run the solver, do forward tracing and plot the results
dpf = yadpf_visolve(dpf, 0.95);
dpf = yadpf_vitrace(dpf, x0, xf, 200);
yadpf_plot(dpf, '-');

% Plot the policy matrix
yadpf_pplot(dpf);


