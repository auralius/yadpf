% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Sutton, R. S., & Barto, A. G. (2018). Reinforcement Learning: An 
% Introduction (2nd ed.). The MIT Press.
%

clear
close all
clc

%% -----------------------------------------------------------------------
% Setup the states and the inputs
P = -1.2  : 0.001 : 0.5;
V = -0.07 : 0.0001 : 0.07;
U = [-1 0 1];

% Setup the horizon
n_horizon = 140;

% Initiate the solver
dps = dps_2X_1U(P, V, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);

% Extract meaningful results
dps = forward_trace(dps, [-0.5 0]);

% Do plotting here
plot_results(dps, '-');

% Animate the mountatin car
visualize(dps);

% Draw the reachability plot
reachability_plot_2X(dps, [0.5 0], 0.1)

%% -----------------------------------------------------------------------
function [p_next, v_next] = state_update_fn(p, v, u, ~)
v_next = v + 0.001*u - 0.0025*cos(3*p);
p_next = p + v_next;

% Hitting the right wall
%[r,c] = find(p(:,:) >= 0.5);
%v_next(r,c) = 0;

% Hitting the left wall
[r,c] = find(p(:,:) <= -1.2);
v_next(r,c) = 0.001; %  inelastic wall, but with positive small number velocity, otherwise it stucks forever

end
%% -----------------------------------------------------------------------
function J = stage_cost_fn(x1, x2, u, k, ~)
xf = 0.5;
vf = 0;

r1 = 1000;
r2 = 1000;
r3 = 5;

J = r1*(xf-x1).^2 + r2*(x2-vf).^2 + r3*u.^2;
end

%% -----------------------------------------------------------------------
function J = terminal_cost_fn(x1, x2)
xf = 0.5;
vf = 0;

r1 = 1000;
r2 = 1000;

J =  r1*(xf-x1).^2 + r2*(x2-vf).^2;
end

%% -----------------------------------------------------------------------
function visualize(dps)
hfig = figure;
hold on;

% Use Universitas Pertamina's logo for the car
up_logo = dlmread('up_tree_logo.mat'); 

% Draw y = 0.4*sin(3p)to represent a mountain
p = -1.2:0.001:0.5;
plot(p, 0.4*sin(3 * p), 'LineWidth', 2);    
set(gca, 'YTickLabel', [ ]); 
xlim([-1.2 0.7])
axis equal

htext1 = text(-0.7,0.5, '');
htext2 = text(-0.7,0.4, '');

% Animate the car
car = plot(0,0, '.r');
for k = 1 : length(dps.x1_star)
    % Normal angle to orient the logo
    theta = atan(3*0.4*cos(3 * dps.x1_star(k)));
    set(car, 'XData', (cos(theta)*up_logo(:,1)-sin(theta)*up_logo(:,2))+...
        dps.x1_star(k));
    
    % Plug the x1_star to the equation of the mountain    
    set(car, 'YData', (sin(theta)*up_logo(:,1)+cos(theta)*up_logo(:,2))+...
        0.4*sin(3 *dps.x1_star(k))); 
        
    htext1.String = ['Stage-', num2str(k), ', x = ', ...
        num2str(dps.x1_star(k)),', v = ', num2str(dps.x2_star(k))] ;
    
    if k < dps.n_horizon % input is one-data-point shorter
        htext2.String = ['u = ', num2str(dps.u_star(k))];
    end
    
    write2gif(hfig, k, 'mountain_car.gif', 0.01);
    drawnow; 
end
end