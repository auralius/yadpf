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

dpf.states           = {P V};
dpf.inputs           = {U};
dpf.T_ocp            = 1;
dpf.T_dyn            = 1;
dpf.n_horizon        = n_horizon;
dpf.state_update_fn  = @state_update_fn;
dpf.stage_cost_fn    = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [-0.5 0]); % Initial state: [-0.5 0]
yadpf_plot(dpf, '-');

% Animate the mountatin car
visualize(dpf);

% Draw the reachability plot, this may take quite some time
% yadpf_rplot(dpf, [0.5 0], 0.1); 

%% -----------------------------------------------------------------------
function X = state_update_fn(X, U, ~)
X{2} = X{2} + 0.001*U{1} - 0.0025*cos(3*X{1});
X{1} = X{1} + X{2};

% Hitting the right wall
%[r,c] = find(X{1}(:,:) >= 0.5);
%X{2}(r,c) = 0;

% Hitting the left wall
[r,c] = find(X{1}(:,:) <= -1.2);
X{2}(r,c) = 0.001; %  inelastic wall, but with positive small number velocity, otherwise it stucks forever

end
%% -----------------------------------------------------------------------
function J = stage_cost_fn(X, U, k, ~)
xf = [0.5 0];  % Terminal state

r1 = 1000;
r2 = 1000;
r3 = 5;

J = r1*(X{1}-xf(1)).^2 + r2*(X{2}-xf(2)).^2 + r3*U{1}.^2;
end

%% -----------------------------------------------------------------------
function J = terminal_cost_fn(X)
xf = [0.5 0];  % Terminal state

r1 = 1000;
r2 = 1000;

J =  r1*(X{1}-xf(1)).^2 + r2*(X{2}-xf(2)).^2;
end

%% -----------------------------------------------------------------------
function visualize(dpf)
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
for k = 1 : length(dpf.x_star{1})
    % Normal angle to orient the logo
    theta = atan(3*0.4*cos(3 * dpf.x_star{1}(k)));
    set(car, 'XData', (cos(theta)*up_logo(:,1)-sin(theta)*up_logo(:,2))+...
        dpf.x_star{1}(k));
    
    % Plug the x1_star to the equation of the mountain    
    set(car, 'YData', (sin(theta)*up_logo(:,1)+cos(theta)*up_logo(:,2))+...
        0.4*sin(3 *dpf.x_star{1}(k))); 
        
    htext1.String = ['Stage-', num2str(k), ', x = ', ...
        num2str(dpf.x_star{1}(k)),', v = ', num2str(dpf.x_star{2}(k))] ;
    
    if k < dpf.n_horizon % input is one-data-point shorter
        htext2.String = ['u = ', num2str(dpf.u_star{1}(k))];
    end
    
    write2gif(hfig, k, 'mountain_car.gif', 0.01);
    drawnow; 
end
end