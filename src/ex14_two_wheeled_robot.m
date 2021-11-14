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

% Setup the states and the inputs
X     =  0  : 0.01  : 1;
Y     = -1  : 0.01  : 0;
THETA = -pi/2 : 0.001  : 0; % Use this if your system has 64 GB of RAM 
%THETA = -pi/2 : 0.01  : 0; 
VL    = [-1 -0.5 0 0.5 1];   % Left-wheel speed
VR    = [-1 -0.5 0 0.5 1];   % Right-wheel speed

% Setup the horizon
T_ocp = 0.2;            % Temporal discretization step
Tf = sqrt(2);           % Completion time
t = 0 : T_ocp : Tf;
n_horizon = length(t);

% Initiate and run the solver
dpf.states = {X Y THETA};
dpf.inputs = {VL VR};
dpf.T_ocp = T_ocp;
dpf.T_dyn = 0.01;
dpf.n_horizon = length(t);
dpf.state_update_fn = @state_update_fn;
dpf.stage_cost_fn = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forward tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [0 0 0]);
yadpf_plot(dpf, '-');

% Additional plotting
visualize(dpf)

%% ========================================================================
function X = state_update_fn(X, U, dt)
L = 0.2;            % Distace between the two wheels
omega = (U{2}-U{1})/L;
v = (U{1}+U{2})/2;

X{1} = dt*v.*cos(X{3})+X{1};
X{2} = dt*v.*sin(X{3})+X{2};

X{3} = dt*omega+X{3};
end

%% ========================================================================
function J = stage_cost_fn(X, U,  k, dt)
r = 10;             % Weighting factors
J = r*dt*(U{1}.^2 + U{2}.^2 );
end

%% ========================================================================
function J = terminal_cost_fn(X)
r = 100;            % Weighting factors
xf = [1 -1 -pi/2];  % Final states

J = r*(X{1}-xf(1)).^2 + r*(X{2}-xf(2)).^2 + r*(X{3}-xf(3)).^2;
end

%% ========================================================================
function visualize(dpf)
hfig = figure;
hold on;

plot(dpf.x_star{1}, dpf.x_star{2}, '-', 'LineWidth', 2);
xlabel('x')
ylabel('y')
axis equal

% This is the car, a simple rectangle
d = 0.02;
rx = [-2*d -2*d 2*d 2*d -2*d];
ry = [-d d d -d -d];
hcar = plot(rx, ry, 'r', 'LineWidth',2);
xlim([min(dpf.x_star{1})-2*d max(dpf.x_star{1})+2*d])
ylim([min(dpf.x_star{2})-2*d max(dpf.x_star{2})+2*d])

for k = 1:length(dpf.x_star{1})
    theta = dpf.x_star{3}(k);
    px = [cos(theta) -sin(theta)] * [rx;ry];
    py = [sin(theta)  cos(theta)]* [rx;ry];
    set(hcar, 'XData', px + dpf.x_star{1}(k));
    set(hcar, 'YData', py + dpf.x_star{2}(k));    
    drawnow;
    write2gif(hfig, k, 'wheeled_robot.gif', 0.3);
end
end