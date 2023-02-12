% Auralius Manurung (manurung.auralius@gmail.com)
% ME - Universitas Pertamina
%
% January 2022
%
% Lee, H. W. J., Teo, K. L., & Rehbock, V. (1998). Suboptimal local +
% feedback control for a class of constrained discrete time nonlinear
% control problems. Computers & Mathematics with Applications, 36(10–12),
% 133–148. https://doi.org/10.1016/S0898-1221(98)80016-3
%
% See page 145 Section 6
%
% also
%
% K.L. Teo, C.J. Goh and K.H. Wong, A Unified Computational Approach to
% Optimal Control Problems, Longman Scientific and Technical, (1991)
% See Chapter 11
%

clear
close all
clc

%%
function X = state_update_fn(X, U, dt)
g = 0.01; % km/s^2
V = 2;    % constant gas nozzle velocity
Q = 0.05*exp(0.01*X{2}).*(X{3}.^2); % aerodynamic drag

X{1} = X{1} - U{1}*dt;
X{2} = X{2} + X{3}*dt;
X{3} = X{3} + ((V*U{1}-Q)./X{1} - g)*dt;
end

%%
function J = stage_cost_fn(X, U, ~, dt)
J =  dt*(10*abs(X{1} - 0.2)- 0.01*X{2}); % maintain the mass at 0.2
end

%%
function J = terminal_cost_fn(X)
J = abs(X{1} - 0.2) - 0.01*X{2} ; % negative weight means "maximize"
end

% ------------------------------------------------------------------------------
% Setup the grids for the states and the inputs
X1 = 0.1 : 0.01   : 1;     % Mass (kg)
X2 = 0   : 0.01   : 30;    % Altitude (km)
X3 = 0   : 0.01   : 1.5;   % Velocity (km/h)
U  = 0   : 0.01   : 0.04;  % Fuel mass-flow rate

% Setup the horizon
T_ocp = 1;
Tf = 100;
t = 0 : T_ocp : Tf;
n_horizon = length(t);

% Initiate the solver
dpf.states              = {X1 X2 X3};
dpf.inputs              = {U};
dpf.T_ocp               = T_ocp;
dpf.T_dyn               = 0.02;
dpf.n_horizon           = length(t);
dpf.state_update_fn     = @state_update_fn;
dpf.stage_cost_fn       = @stage_cost_fn;
dpf.terminal_cost_fn    = @terminal_cost_fn;

% Initiate and run the solver, do forward tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [1 0 0]);
yadpf_plot(dpf, '-');

