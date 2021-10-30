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

global Ts v;

% Define the Dubin's car parameters
R = 0.5; % lower bound for the turning radius
v = 1;   % the constant speed of the car 

% Setup the states and the inputs
X     =  0   : 0.01 : 2;
Y     = -2   : 0.01 : 2;
THETA = -pi  : 0.01 : pi;
OMEGA = [-v/R 0 v/R];

% Setup the horizon
Ts = 0.1;            % Temporal discretization step
Tf = 4;              % Completion time
t = 0:Ts:Tf;
n_horizon = length(t);

% Initiate and run the solver, do forward tracing for a given IC, and plot
% the results
dps = dps_3X_1U(X, Y, THETA, OMEGA, n_horizon, @state_update_fn,...
                @stage_cost_fn, @terminal_cost_fn);
dps = forward_trace(dps, [0 0 0]);
plot_results(dps, '-');

% Additional plotting
visualize(dps);

%% ------------------------------------------------------------------------
function [x_next, y_next, theta_next] = state_update_fn(x, y, theta, omega)
global Ts v;

x_next = Ts*v*cos(theta)+x;
y_next = Ts*v*sin(theta)+y;
theta_next = Ts*v*omega+theta;
end

%% ------------------------------------------------------------------------
function J = stage_cost_fn(x, y, theta, omega, k)
global Ts;
J = 10*Ts * omega.^2;
end

%% ------------------------------------------------------------------------
function J = terminal_cost_fn(x, y, theta)
% Weighting factors
r = 1000;

% Final states
xf = 0.6;
yf = -0.6;
thetaf = -pi/2; 

% Calculate the cost
J = r*(x-xf).^2 + r*(y-yf).^2 + r*(theta-thetaf).^2;
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

for k = 1:dps.n_horizon
    theta = dps.x3_star(k);
    px = [cos(theta) -sin(theta)] * [rx;ry];
    py = [sin(theta)  cos(theta)]* [rx;ry];
    set(hcar, 'XData', px + dps.x1_star(k));
    set(hcar, 'YData', py + dps.x2_star(k));    
    drawnow;
    write2gif(hfig, k, 'dubins_car.gif', 0.3);
end
end