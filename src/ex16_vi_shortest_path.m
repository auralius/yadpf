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

global T terra_size;

% Terrain1.mat, from 1 to 4225 (64x64)
% Terrain2.mat, from 1 to 16641 (129x129)
load ('Terrain1.mat');
%load ('Terrain2.mat');
terra_size = size(T);

% Setup the states and the inputs
X  = 1 : 1 : length(T);
Y  = 1 : 1 : length(T);
Ux  = [-1 0 1];
Uy  = [-1 0 1];

% Prepare the structure
dpf.states           = {X Y};
dpf.inputs           = {Ux Uy};
dpf.T_ocp            = 1;
dpf.T_dyn            = 1;
dpf.max_iter         = 15000;
dpf.state_update_fn  = @state_update_fn;
dpf.stage_cost_fn    = @stage_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_visolve(dpf, 0.99);
dpf = yadpf_vitrace(dpf, [1 1], [50 60]); % From [1 1] to [50 60]
yadpf_plot(dpf, '-');

% Additional plotting
figure
hold on
surf(T');
plot3(dpf.x_star{1}, dpf.x_star{2}, T(sub2ind(terra_size, ...
      dpf.x_star{1}, dpf.x_star{2})), '-r', 'LineWidth',2);
view([-46.1 49.7]);
xlabel('x')
ylabel('y')
axis equal

%% The state update function
function X = state_update_fn(X, U, ~)
X{1} = X{1} + U{1};
X{2} = X{2} + U{2};
end

%% The stage cost function
function J = stage_cost_fn(X, U, k, ~)
global terra_size;

r = terra_size(1);
c = terra_size(2);

% First, check where the car's next position for a selected input
X_to = state_update_fn(X, U);
X_to{1} = min(max(X_to{1},1),r);
X_to{2} = min(max(X_to{2},1),c);

% Next, compute the height difference of the current position to the next
% position
dh = get_height_difference(X{1}, X{2}, X_to{1}, X_to{2});

% Target / final states
xf = [50 60];

% Finally, compute the cost
k1 = 10000;
k2 = 0.5;
k3 = 0.5;

J =  k1*max(dh,0) + k2*(X{1}-xf(1)).^2 + k3.*(X{2}-xf(2)).^2;
end

%% This function computes the height differece between two location 
%  coordinates
function dh = get_height_difference(x_from, y_from, x_to, y_to)
global T terra_size;

r = terra_size(1);
c = terra_size(2);

to_id = sub2ind([r c], x_to, y_to);
from_id = sub2ind([r c], x_from, y_from);

dh = T(to_id) - T(from_id);

end