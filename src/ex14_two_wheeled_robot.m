% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Simple two-wheeled differential drive robot
%
% http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
% 
% Dudek, G. & Jenkin, M. R. M. (2000), Computational principles of mobile 
% robotics. , Cambridge University Press .
%
% =========================================================================
% CAUTION:
% This simulation requires at least 32 GB of RAM !
% =========================================================================

clear
close all
clc

global L;

L = 0.2; % Distace between the two wheels

% Setup the states and the inputs
X     =  0  : 0.01  : 1;
Y     = -1  : 0.01  : 0;
THETA = -pi/2 : 0.001  : 0; % Use this if your system has 64 GB of RAM 
%THETA = -pi/2 : 0.01  : 0; 
VL    = [-1 -0.5 0 0.5 1];   % Left-wheel speed
VR    = [-1 -0.5 0 0.5 1];   % Right-wheel speed

% Setup the horizon
T_ocp = 0.2;            % Temporal discretization step
Tf = sqrt(2);              % Completion time
t = 0 : T_ocp : Tf;
n_horizon = length(t);

% Initiate and run the solver
dps = dps_3X_2U(X, Y, THETA, VL, VR, n_horizon, @state_update_fn, ...
                @stage_cost_fn, @terminal_cost_fn, T_ocp, 0.01);

% Extract meaningful results
dps = forward_trace(dps, [0 0 0]);

% Do plotting here
plot_results(dps, '-');

% Additional plotting
visualize(dps)

%% ========================================================================
function [x_next, y_next, theta_next] = state_update_fn(x, y, theta, ...
                                                       vl, vr, dt)
global L;

omega = (vr-vl)/L;
v = (vr+vl)/2;

x_next = dt*v.*cos(theta)+x;
y_next = dt*v.*sin(theta)+y;

theta_next = dt*omega+theta;
end

%% ========================================================================
function J = stage_cost_fn(x, y, theta, vl,vr,  k, dt)
% Weighting factors
r = 10;
J = r*dt*(vl.^2 + vr.^2 );
end

%% ========================================================================
function J = terminal_cost_fn(x, y, theta)
% Weighting factors
r = 100;

% Final states
xf = 1;
yf = -1;
thetaf = -pi/2; 

% Calculate the cost
J = r*(x-xf).^2 + r*(y-yf).^2 + r*(theta-thetaf).^2;
end

%% ========================================================================
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
    write2gif(hfig, k, 'wheeled_robot.gif', 0.3);
end
end