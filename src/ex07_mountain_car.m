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
V = -0.07 : 0.001 : 0.07;
U = [-1 0 1];

% Setup the horizon
n_horizon = 200;

% Initiate the solver
dps = dps_2X_1U(P, V, U, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn);

% Extract meaningful results
dps = forward_trace(dps, [-0.6 0]);

% Do plotting here
plot_results(dps, '-');

% Animate the mountatin car
visualize(dps);

% Draw the reachability plot
reachability_plot_2X(dps, [0.5 0], 0.02)

%% -----------------------------------------------------------------------
function [p_next, v_next] = state_update_fn(p, v, u)
v_next = v + 0.001*u - 0.0025*cos(3*p);
p_next = p + v;
end

%% -----------------------------------------------------------------------
function J = stage_cost_fn(x1, x2, u, k)
xf = 0.5;
vf = 0;

J = u.^2 + (x1-xf).^2 + (x2-vf).^2;
end

%% -----------------------------------------------------------------------
function J = terminal_cost_fn(x1, x2)
% Weighting factors
J = zeros(size(x1));
end

%% -----------------------------------------------------------------------
function visualize(dps)
hfig = figure;
hold on;

% Draw y = sine(3p)to representate a mountain
p = -1.2:0.001:0.5;
plot(p, 0.4*sin(3 * p), 'LineWidth', 2);    
set(gca, 'YTickLabel', [ ]); 
axis equal

htext1 = text(-0.7,0.5, '');
htext2 = text(-0.7,0.4, '');

% Animate the car
car = plot(0,0, 'or', 'LineWidth', 4);
for k = 1 : dps.n_horizon
    set(car, 'XData', dps.x1_star(k));
    
    % Plug the XStar to the equation of the mountain
    set(car, 'YData', 0.4*sin(3*dps.x1_star(k))); 
    
    htext1.String = ['Stage-', num2str(k), ', x = ', ...
        num2str(dps.x1_star(k)),', v = ', num2str(dps.x2_star(k))] ;
    
    if k < dps.n_horizon % input is one-data-point shorter
        htext2.String = ['u = ', num2str(dps.u_star(k))];
    end
    
    write2gif(hfig, k, 'mountain_car.gif', 0.01);
    drawnow; 
end
end