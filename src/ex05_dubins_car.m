% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Dubin's car
%
% Jonsson, U. (2010). Optimal Control. 
% See Example 2, page 3
% https://www.math.kth.se/optsyst/grundutbildning/kurser/SF2852/LecturenotesScanned.pdf

clear
close all
clc

global Ts;

% Define the Dubin's car parameters
R = 1;

% Setup the states and the inputs
X     = 0    : 0.01 : 2;
Y     = -1   : 0.01 : 0;
THETA = -2*pi: 0.01 : 2*pi;
OMEGA = [-1/R 0 1/R];

% Setup the horizon
Ts = 0.1;            % Temporal discretization step
Tf = pi/2+Ts;        % Until completed, theoritically
t = 0:Ts:Tf;
n_horizon = length(t);

% Initiate the solver
tic
dps = dps_3X_1U(X, Y, THETA, OMEGA, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);
toc

% Extract meaningful results
dps = forward_trace(dps, [0 0 0]);

% Do plotting here
plot_results(dps, '--d');

% Additional plotting
figure
plot(dps.x1_star, dps.x2_star, '-o', 'LineWidth', 2);
xlabel('x')
ylabel('y')
axis equal


%%
function [x_next, y_next, theta_next] = state_update_fn(x, y, theta, omega)
global Ts;

x_next = Ts.*cos(theta)+x;
y_next = Ts.*sin(theta)+y;
theta_next = Ts.*omega+theta;
end

%%
function J = stage_cost_fn(x, y, theta, omega, k)
J = zeros(size(omega));
end

%%
function J = terminal_cost_fn(x, y, theta)
% Weighting factors
r = 100;

% Final states
xf = 1;
yf = -1;
thetaf = -pi/2; 

% Calculate the cost
J = r.*(x-xf).^2 + r.*(y-yf).^2 + r.*(theta-thetaf).^2;
end