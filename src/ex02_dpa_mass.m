% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% A mass (1 kg) is moving from x=0 to x=0.5 in exactly 1 second because of 
% an exteral force. The damping coefficient is 0.1. At the destination, the
% mass must stop moving. The external force is bounded (-4 N to 4 N).
%

%%

clear
close all
clc

% Setup the states and the inputs
X = 0  : 0.001 : 1; % Position
V = 0  : 0.001 : 1; % Velocity
F = -4 : 0.1   : 4; % Applied force

% Setup the horizon
Tf = 1;          % 1 second
T_ocp = 0.1;     % Temporal discretization step
t  = 0 : T_ocp : Tf;

dpf.states           = {X, V};
dpf.inputs           = {F};
dpf.T_ocp            = T_ocp;
dpf.T_dyn            = 0.01;        % Time step for the dynamic simulation         
dpf.n_horizon        = length(t);
dpf.state_update_fn  = @state_update_fn;
dpf.stage_cost_fn    = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forward tracing for the given initial 
% condition and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [0 0]); % Initial state: [0 0]
yadpf_plot(dpf, '-');

% Optional: draw the reachability plot
yadpf_rplot(dpf, [0.5 0], 0.1);

%% The state unpdate funtion 
function X = state_update_fn(X, F, dt)
m = 1;   % Mass
b = 0.1; % Damping coefficient
 
X{1} = X{1} + dt*X{2};
X{2} = X{2} - b/m*dt.*X{2} + dt/m.*F{1};
end

%% The stage cost function
function J = stage_cost_fn(X, F, k, dt)
J = dt*F{1}.^2;
end

%% The terminal cost function
function J = terminal_cost_fn(X)
xf = [0.5 0];

% Control gains
r1 = 1000;
r2 = 100;

J = r1*(X{1}-xf(1)).^2 + r2*(X{2}-xf(2)).^2;
end