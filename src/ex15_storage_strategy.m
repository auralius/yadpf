%% 
% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% Optimal storage strategy
%
% Jonsson, U. (2010). Optimal Control. 
% See Example 1, page 2
% https://www.math.kth.se/optsyst/grundutbildning/kurser/SF2852/LecturenotesScanned.pdf
%
%%
clear
close all
clc

% Setup the states and the inputs
X = 0 : 0.1 : 500;
U = 0 : 0.1 : 10;

% Setup the horizon
T_ocp = 1;                % Every 1 day
T_vect = 0 : T_ocp : 30;  % Day-0 to day-30
n_horizon = length(T_vect);

x0 = 0;                   % Initial state
xf = 200;                 % Terminal state

dpf.states = {X};
dpf.inputs = {U};
dpf.T_ocp = T_ocp;
dpf.T_dyn = T_ocp;
dpf.n_horizon = n_horizon;
dpf.state_update_fn = @state_update_fn;
dpf.stage_cost_fn = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, x0);
yadpf_plot(dpf, '-');

% Reachability plot
yadpf_rplot(dpf, xf, 10);

%% ------------------------------------------------------------------------
function X = state_update_fn(X, U, dt)
X{1} = dt*U{1} + X{1}; 
end

%% ------------------------------------------------------------------------
function J = stage_cost_fn(X, U, k, dt)
r = 0.1; % production cost growth rate
c = 10;  % storage cost per time unit
J =  dt*exp(r*k*dt)*U{1} + c*X{1}*dt;
end

%% ------------------------------------------------------------------------
function J = terminal_cost_fn(X)
xf = 200; 
L = 100;
J = L .* (xf-X{1}).^2;
end