% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% A mass has two inputs: 
%  U1: for moving to the right and for moving to the right, 0N to 5N
%  U2: for moving to the right and for moving to the left, 0N to 4N
%

clear
close all
clc

% Setup the states and the inputs
X1 =  0 : 0.001 : 1;
X2 =  0 : 0.01  : 1;
U1 =  0 : 0.1   : 5;
U2 = -4 : 0.1   : 0;

% Setup the horizon
Tf = 1;     % 1 second
dt = 0.2;   % Temporal discretization step
t = 0:dt:Tf;
n_horizon = length(t);

% Initiate the solver
dps = dps_2X_2U(X1, X2, U1, U2, n_horizon, @state_update_fn, @stage_cost_fn, ...
                @terminal_cost_fn, dt, 0.01);

% Extract meaningful results
dps = forward_trace(dps, [0 0]);

% Do plotting here
plot_results(dps, '-');

%%
function [x1_next, x2_next] = state_update_fn(x1, x2, u1, u2, dt)
m = 1;
 
x1_next = x1 + dt*x2;
x2_next = x2 + dt/m.*(u1+u2);

end

%%
function J = stage_cost_fn(x1, x2, u1, u2, k, dt)
a1 = 1;
a2 = 1;
J = a1*dt*u1.^2 + a2*dt*u2.^2;
end

%%
function J = terminal_cost_fn(x1, x2)
% Weighting factors
a2 = 1000;
a3 = 1000;

% Final states
xf = 0.5;
vf = 0;

J = a2.*(x1-xf).^2 + a3.*(x2-vf).^2;
end