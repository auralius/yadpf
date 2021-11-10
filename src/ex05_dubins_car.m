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

clear
close all
clc

global v xf;

% Define the Dubin's car parameters
R = 0.5; % lower bound for the turning radius
v = 1;   % the constant speed of the car 

% Setup the states and the inputs
X     = -1    : 0.01 : 1;
Y     = -1    : 0.01 : 0;
THETA = -2*pi : 0.01 : 0;
OMEGA = [-v/R 0 v/R];

% Setup the horizon
Tocp = 0.2; % Temporal discretization step
Tf = pi;    % Theoritical completion time, otherwise the car will go around-and-around
t = 0 : Tocp : Tf;
n_horizon = length(t);

% Initial and terminal states
% We are going to make a full cricle of radius R
x0 = [0 0 0];
xf = [0 0 -2*pi];

% Initiate and run the solver, do forward tracing for a given IC, and plot
% the results
dps = dps_3X_1U(X, Y, THETA, OMEGA, n_horizon, @state_update_fn,...
                @stage_cost_fn, @terminal_cost_fn, Tocp, 0.01);
dps = forward_trace(dps, x0);
plot_results(dps, '-');

% Additional plotting
visualize(dps);

%% ------------------------------------------------------------------------
function [x_next, y_next, theta_next] = state_update_fn(x, y, theta, omega, dt)
global v;

x_next = dt*v*cos(theta)+x;
y_next = dt*v*sin(theta)+y;
theta_next = dt*v*omega+theta;
end

%% ------------------------------------------------------------------------
function J = stage_cost_fn(x, y, theta, omega, k, dt)
J = dt * omega.^2;  % Minimize the input
end

%% ------------------------------------------------------------------------
function J = terminal_cost_fn(x, y, theta)
global xf;
% Weighting factors
r = 1000;

% Calculate the cost
J = r*(x-xf(1)).^2 + r*(y-xf(2)).^2 + r*(theta-xf(3)).^2;
end

%% ------------------------------------------------------------------------
function visualize(dps)
hfig = figure;
hold on;

plot(dps.x1_star, dps.x2_star, '-', 'LineWidth', 2);
xlabel('x')
ylabel('y')
axis equal

% This is the car, a simple rectangle
d = 0.02;
rx = [-2*d -2*d 2*d 2*d -2*d];
ry = [-d d d -d -d];
hcar = plot(rx, ry, 'r', 'LineWidth',2);
xlim([min(dps.x1_star)-2*d max(dps.x1_star)+2*d])
ylim([min(dps.x2_star)-2*d max(dps.x2_star)+2*d])

for k = 1:length(dps.x1_star)
    theta = dps.x3_star(k);
    px = [cos(theta) -sin(theta)] * [rx;ry];
    py = [sin(theta)  cos(theta)]* [rx;ry];
    set(hcar, 'XData', px + dps.x1_star(k));
    set(hcar, 'YData', py + dps.x2_star(k));    
    drawnow;
    write2gif(hfig, k, 'dubins_car.gif', 0.01);
end
end