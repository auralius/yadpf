%%
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Find the shortest path in a terrain

%%
clear
close all
clc

global X Y Ux Uy T terrain_size;

% Terrian1.mat, from 1 to 4225 (64x64)
% Terrian2.mat, from 1 to 16641 (129x129)
%load ('Terrain1.mat');
load ('Terrain2.mat');
terrain_size = size(T);

% Setup the states and the inputs
X  = ( 1 : 1 : length(T))';
Y  = ( 1 : 1 : length(T))';
Ux  = [-1 0 1]';
Uy  = [-1 0 1]';

%  The horizon isn't yet known, let us just guess
n_horizon = 150;

% Initiate the solver
dps = dps_2X_2U(X, Y, Ux, Uy, n_horizon, @state_update_fn, ...
                @stage_cost_fn, @terminal_cost_fn);

% Extract meaningful results for a given initial condition
x_ic = 1;
y_ic = 1;
dps = trace_2X_2U(dps, x_ic, y_ic);

% Do plotting here
plot_2X_2U(dps, '-');

% Additional plotting
figure
hold on
surf(T');
plot3(dps.x1_star, dps.x2_star, T(sub2ind(terrain_size, ...
      dps.x1_star, dps.x2_star)), '-r', 'LineWidth',2);
xlabel('x')
ylabel('y')
axis equal

%%
function [x_next, y_next] = state_update_fn(x, y, ux, uy)
x_next = x + ux;
y_next = y + uy;
end

%%
function J = stage_cost_fn(x, y, ux, uy, k)
global terrain_size;

r = terrain_size(1);
c = terrain_size(2);

[x_to, y_to] = state_update_fn(x, y, ux, uy);

x_to = min(max(x_to,1),r);
y_to = min(max(y_to,1),c);

dh = get_height_difference(x, y, x_to, y_to);

w = 2;
J = ux.^2 + uy.^2 + w.*max(dh,0);

end

%%
function J = terminal_cost_fn(x, y)
% Weighting factors
k1 = 100;
k2 = 100;

% Final states
xf = 65;
yf = 80;

J = k1.*(x-xf).^2 + k2.*(y-yf).^2;
end

%%
function dh = get_height_difference(x_from, y_from, x_to, y_to)
global T terrain_size;

r = terrain_size(1);
c = terrain_size(2);

to_id = sub2ind([r c], x_to, y_to);
from_id = sub2ind([r c], x_from, y_from);

dh = T(to_id) - T(from_id);
end

