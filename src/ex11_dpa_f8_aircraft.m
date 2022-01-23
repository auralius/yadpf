% Auralius Manurung (manurung.auralius@gmail.com)
% ME - Universitas Pertamina
% November 2021
%
% Stabilization of an F8 aircraft
%
% Banks, S. P., & Mhana, K. J. (1992). Optimal control and stabilization 
% for nonlinear systems. IMA Journal of Mathematical Control and 
% Information, 9(2), 179–196. https://doi.org/10.1093/imamci/9.2.179
%
% Kaya, C. Y., & Noakes, J. L. (2003). Computational Method for 
% Time-Optimal Switching Control. Journal of Optimization Theory and 
% Applications, 117(1), 69–92. https://doi.org/10.1023/A:1023600422807
%
% Garrard, W. L., & Jordan, J. M. (1977). Design of nonlinear automatic 
% flight control systems. Automatica, 13(5), 497–505. 
% https://doi.org/10.1016/0005-1098(77)90070-X
%
% Kaya, C. Y., & Noakes, J. L. (1996). Computations and time-optimal 
% controls. Optimal Control Applications and Methods, 17(3), 171–185. 
% https://doi.org/10.1002/(SICI)1099-1514(199607/09)17:3<171::AID-OCA571>3.0.CO;2-9
%
% This may require at leasr 16GB of RAM!
%

clear
close all
clc

% Setup the grids for the states and the inputs
X1 = -0.2 : 0.01  : 0.6;   % Attack angle : alpha
X2 = -0.6 : 0.01  : 0.3;   % Pitch angle : theta
X3 = -1.2 : 0.01  : 0.7;   % Pitch rate : theta_dot
U  = deg2rad([-3 0 3]);    % Tail deflection

% Setup the horizon
T_ocp = 0.2;  % Discretization time step, less than 0.3s requires finer grid
Tf = 10;     % From the paper, Tf < 7s, 10s should be fine
t = 0 : T_ocp : Tf;
n_horizon = length(t);

% Initiate the solver
dpf.states              = {X1 X2 X3};
dpf.inputs              = {U};
dpf.T_ocp               = T_ocp;
dpf.T_dyn               = 0.01;
dpf.n_horizon           = length(t);
dpf.state_update_fn     = @state_update_fn;
dpf.stage_cost_fn       = @stage_cost_fn;
dpf.terminal_cost_fn    = @terminal_cost_fn;

% Initiate and run the solver, do forwar tracing and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [deg2rad(26.7) 0 0]);
yadpf_plot(dpf, '-');
%%
function X = state_update_fn(X, U, dt)

X{1} = (-0.877*X{1} + X{3} - 0.088*X{1}.*X{3} + 0.47*X{1}.^2 - ...
        0.019*X{2}.^2 - X{1}.^2.*X{3} + 3.846*X{1}.^3 - 0.215*U{1} + ...
        0.28*X{1}.^2.*U{1} + 0.47*X{1}.*U{1}.^2 + 0.63*U{1}.^3)*dt + X{1};
X{2} = X{3}*dt + X{2};
X{3} = (-4.208*X{1} - 0.396*X{3} - 0.47*X{1}.^2 - 3.564*X{1}.^3 - ...
        20.967*U{1} + 6.265*X{1}.^2.*U{1} + 46*X{1}.*U{1}.^2 + ...
        61.4*U{1}.^3)*dt + X{3};
end

%%
function J = stage_cost_fn(X, U, ~, dt)
% Tuning parameters q and r
q1 = 0.25; % attack angle
q2 = 0.25; % pitch
q3 = 0.25; % pitch rate
r  = 1;   % input can be zero, so we apply input minimizer, but not much

J = dt.*(q1*(X{1}.^2) + q2*(X{2}.^2) + q3*(X{3}.^2) + r*U{1}.^2);
end

%%
function J = terminal_cost_fn(X)
J = 0.25*((X{1}.^2) + (X{2}.^2) + (X{3}.^2) );
end