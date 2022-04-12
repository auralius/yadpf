%%
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Piecewise mass-spring system hanging from two anchor points
% Model as a planar kinematic car
% https://www.mathworks.com/help/optim/ug/problem-based-cone-programming-springs.html
%
%%
clc;
close all;
clear

% Setup the states and the inputs
X   = 0 :  0.02  : 5;
Y   = 1 :  0.02  : 5;
U  = 0  :  0.02  : 2;
V  = -2 :  0.02  : 2;

% Initiate the solver
dpf.states           = {X, Y};
dpf.inputs           = {U, V};
dpf.T_ocp            = 1;
dpf.T_dyn            = 0.1;
dpf.n_horizon        = 7;
dpf.state_update_fn  = @state_update_fn;
dpf.stage_cost_fn    = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [0 5]);
yadpf_plot(dpf, '-');

% Additional plotting
figure
hold on;
plot(dpf.x_star{1}, dpf.x_star{2}, '-', 'LineWidth', 2);
plot(dpf.x_star_unsimulated{1}, dpf.x_star_unsimulated{2}, '*r', 'LineWidth', 2);
xlabel('x')
ylabel('y')

%% The state update function
function X = state_update_fn(X, U, dt)
X{1} = X{1} + U{1} * dt;
X{2} = X{2} + U{2} * dt;
end

%% The stage cost function
function J = stage_cost_fn(X, U, i, ~)
K = 40*(1:6);
L = [1 1/2 1 2 1 1/2];
% The last mass is not important, since the terminal node is "fixed".
M = [2 1 3 2 1 0];   
g = 9.807;

d = sqrt(U{1}.^2 + U{2}.^2);      
T = 0.5 * K(i) * (d-L(i)).^2; % Energy by the spring
V = (M(i) * g) .* X{2};       % Energy by the gravity
J = T + V;                    % Total energy to minimize  
end

%% The terminal cost function
function J = terminal_cost_fn(X)
% Control gains for the terminal node
k1 = 300;
k2 = 300;

% Targetted terminal states
xf = [5 4];

J = k1*(X{1}-xf(1)).^2 + k2*(X{2}-xf(2)).^2;
end