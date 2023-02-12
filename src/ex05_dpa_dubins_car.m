% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Dubin's car
%
% Jonsson, U. (2010). Optimal Control.
% See Example 2, page 3
% https://www.math.kth.se/optsyst/grundutbildning/kurser/SF2852/LecturenotesScanned.pdf
%
%%
clear
close all
clc

global v xf;

%% The state update function
function X = state_update_fn(X, U, dt)
global v;

% X{1} is x, X{2} is y, and X{3} is theta, and U is omega
X{1} = dt*v*cos(X{3})+X{1};
X{2} = dt*v*sin(X{3})+X{2};
X{3} = dt*v*U{1}+X{3};
end

%% The stage cost function
function J = stage_cost_fn(X, U, k, dt)
J = dt * U{1}.^2;  % Minimize the input
end

%% The terminal cost function
function J = terminal_cost_fn(X)
global xf;
% Weighting factors
r = 1000;

% Calculate the cost
J = r*(X{1}-xf(1)).^2 + r*(X{2}-xf(2)).^2 + r*(X{3}-xf(3)).^2;
end

%% Animate the car's motion in XY-space
function visualize(dpf)
hfig = figure;
hold on;

plot(dpf.x_star{1}, dpf.x_star{2}, '-', 'LineWidth', 2);
xlabel('x')
ylabel('y')
axis equal

% This is the car, a simple rectangle
d = 0.02;
rx = [-2*d -2*d 2*d 2*d -2*d];
ry = [-d d d -d -d];
hcar = plot(rx, ry, 'r', 'LineWidth',2);
xlim([min(dpf.x_star{1})-2*d max(dpf.x_star{1})+2*d])
ylim([min(dpf.x_star{2})-2*d max(dpf.x_star{2})+2*d])

for k = 1:length(dpf.x_star{1})
    theta = dpf.x_star{3}(k);
    px = [cos(theta) -sin(theta)] * [rx;ry];
    py = [sin(theta)  cos(theta)]* [rx;ry];
    set(hcar, 'XData', px + dpf.x_star{1}(k));
    set(hcar, 'YData', py + dpf.x_star{2}(k));
    drawnow;
    write2gif(hfig, k, 'dubins_car.gif', 0.01);
end
end

% -----------------------------------------------------------------------------
% Define the Dubin's car parameters
R = 0.5; % lower bound for the turning radius
v = 1;   % the constant speed of the car

% Setup the states and the inputs
X     = -1    : 0.01 : 1;
Y     = -1    : 0.01 : 0;
THETA = -2*pi : 0.01 : 0;
OMEGA = [-v/R 0 v/R];

% Setup the horizon
T_ocp = 0.2; % Temporal discretization step
Tf = pi;    % Theoritical completion time, otherwise the car will go around-and-around
t = 0 : T_ocp : Tf;
n_horizon = length(t);

% Initial and terminal states
% We are going to make a full cricle of radius R
x0 = [0 0 0];
xf = [0 0 -2*pi];

% Build the structure
dpf.states              = {X, Y, THETA};
dpf.inputs              = {OMEGA};
dpf.T_ocp               = T_ocp;
dpf.T_dyn               = 0.01;
dpf.n_horizon           = length(t);
dpf.state_update_fn     = @state_update_fn;
dpf.stage_cost_fn       = @stage_cost_fn;
dpf.terminal_cost_fn    = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, x0);
yadpf_plot(dpf, '-');

% Animation
visualize(dpf);


