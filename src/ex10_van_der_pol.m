% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% The Time-optimal van der Pol equation with a conbtrol input
%
% Kaya, C. Y., & Noakes, J. L. (2003). Computational Method for 
% Time-Optimal Switching Control. Journal of Optimization Theory and 
% Applications, 117(1), 69–92. https://doi.org/10.1023/A:1023600422807
%

clear
close all
clc

% Setup the states and the inputs
X1 = -1   : 0.002 : 2;
X2 = -1   : 0.002 : 1;
U  = [-1 0 1];

% Setup the horizon
T_ocp = 0.1;         % Temporal discretization step
Tf = 4;              % Final time
t = 0 : T_ocp : Tf;

% Initiate the solver
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
dpf = yadpf_trace(dpf, [1 1]);
yadpf_plot(dpf, '-');

% Additional plotting
figure
plot(dpf.x_star{1}, dpf.x_star{2}, '-', 'LineWidth', 2);
xlabel('$x_1$', 'Interpreter','latex')
ylabel('$x_2$', 'Interpreter','latex')
axis equal

%%
function X = state_update_fn(X, U, dt)  
X{1} = dt*X{2} + X{1};
X{2} = dt*(-X{1} - (X{1}.^2-1) .* X{2} + U{1}) + X{2};
end

%%
function J = stage_cost_fn(X, U, k, dt)
xf = [0 0];          % Terminal states

q = 100;
r = 1;
J = dt * (q*(X{1}-xf(1)).^2 + q*(X{2}-xf(2)).^2 + r*U{1}.^2);
end

%%
function J = terminal_cost_fn(X)
xf = [0 0];          % Terminal states

q = 100;
J = q*(X{1}-xf(1)).^2 + q*(X{2}-xf(2)).^2;
end