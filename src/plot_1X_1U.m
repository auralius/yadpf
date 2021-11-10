function plot_1X_1U(dps, line_style)

n =  length(dps.u_star);
tf = length(dps.u_star) * dps.T_dyn;
t = linspace(0, tf, n);

figure

subplot(2,1,1)
hold on;
plot(t, dps.x_star, line_style, 'LineWidth', 2)
line([0 t(end)], [min(dps.X) min(dps.X)],'LineStyle','--', 'Color', 'red')
line([0 t(end)], [max(dps.X) max(dps.X)],'LineStyle','--', 'Color', 'red')
xlabel('Time', 'Interpreter','latex')
ylabel('$x$', 'Interpreter','latex')
xlim([0 t(end)])

subplot(2,1,2)
hold on;
plot(t, dps.u_star, line_style, 'LineWidth', 2)
line([0 t(end)], [min(dps.U) min(dps.U)],'LineStyle','--', 'Color', 'red')
line([0 t(end)], [max(dps.U) max(dps.U)],'LineStyle','--', 'Color', 'red')
xlabel('Time', 'Interpreter','latex')
ylabel('$u(k)$', 'Interpreter','latex')
xlim([0 t(end)])

end