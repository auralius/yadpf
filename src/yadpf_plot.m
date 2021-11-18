function yadpf_plot(dpf, line_style)
% Plot the optimal inputs and the optimal states
%
% Syntax:  yadpf_plot(dpf, line_style)
%
% Inputs:
%    dpf - The data structure for the optimal control problem
%    line_style  - The line style, similar to the line style as in the
%                  ordinary plot command
%
% Outputs: -   
%
% Author:
%   Auralius Manurung
%   Universitas Pertamina 
%   auralius.manurung@ieee.org

%------------- BEGIN CODE --------------
if nargin < 2
    line_style = '-';
end

n =  length(dpf.u_star{1});
tf = n * dpf.T_dyn;
t = linspace(0, tf, n);

figure

total_plots = dpf.n_states + dpf.n_inputs;
active_plot = 1;

for i = 1 : dpf.n_states
    subplot(total_plots, 1, active_plot)
    hold on;
    plot(t, dpf.x_star{i}, line_style, 'LineWidth', 2)
    line([0 t(end)], [dpf.lb(i) dpf.lb(i)],'LineStyle','--', 'Color', 'red')
    line([0 t(end)], [dpf.ub(i) dpf.ub(i)],'LineStyle','--', 'Color', 'red')
    xlabel('t', 'Interpreter','latex')
    ylabel(['$x_' num2str(i) '(t)$'], 'Interpreter','latex')
    xlim([0 t(end)])

    active_plot = active_plot + 1;
end

for i = 1 : dpf.n_inputs
    subplot(total_plots, 1, active_plot)
    hold on;
    stairs(t, dpf.u_star{i}, line_style, 'LineWidth', 2)
    line([0 t(end)], [min(dpf.inputs{i}) min(dpf.inputs{i})],'LineStyle','--', 'Color', 'red')
    line([0 t(end)], [max(dpf.inputs{i}) max(dpf.inputs{i})],'LineStyle','--', 'Color', 'red')
    xlabel('t', 'Interpreter','latex')
    ylabel(['$u_' num2str(i) '(t)$'], 'Interpreter','latex')
    xlim([0 t(end)])

    active_plot = active_plot + 1;
end

end
%------------- END OF CODE --------------
