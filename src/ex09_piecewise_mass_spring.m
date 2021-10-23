%%
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Piecewise mass-spring system hanging from two anchor points
% https://www.mathworks.com/help/optim/ug/problem-based-cone-programming-springs.html

clc;
close all;
clear all;

% Setup the states and the inputs
X   = 0  : 0.05  : 5;
Y   = 0  : 0.05  : 5;
Ux  = 0  : 0.05  : 2;
Uy  = -3 : 0.05  : 3;

% Number of horizons
n_horizon = 7;

% Initiate the solver
dps = dps_2X_2U(X, Y, Ux, Uy, n_horizon, @state_update_fn, ...
                @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results for a given initial condition
x_ic = 0;
y_ic = 5;
dps = forward_trace(dps, [x_ic y_ic]);

% Do plotting here
plot_results(dps, '-d');

% Additional plotting
figure
hold on
plot(dps.x1_star, dps.x2_star, '-*');
xlabel('x')
ylabel('y')

%%
function [x_next, y_next] = state_update_fn(x, y, ux, uy)
x_next = x + ux;
y_next = y + uy;
end

%%
function J = stage_cost_fn(x, y, ux, uy, i)
K = 40*(1:6);
L = [1 1/2 1 2 1 1/2];
% The last mass is not important, since the terminal node is "fixed".
M = [2 1 3 2 1 0];   
g = 9.807;

d = sqrt(ux.^2 + uy.^2);      
T = 0.5 * K(i) * (d-L(i)).^2; % Energy by the spring
V = (M(i) * g) .* y;          % Energy by the gravity
J = T + V;                    % Total energy to minimize  
end

%%
function J = terminal_cost_fn(x, y)
% Control gains for the terminal node
k1 = 1000;
k2 = 1000;

% Final states
xf = 5;
yf = 4;

J = k1.*(x-xf).^2 + k2.*(y-yf).^2;
end