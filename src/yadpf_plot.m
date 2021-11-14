function yadpf_plot(dpf, line_style)

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