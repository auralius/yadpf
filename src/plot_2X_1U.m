function plot_2X_1U(dps, line_style)
n =  length(dps.u_star);
tf = length(dps.u_star) * dps.T_dyn;
t = linspace(0, tf, n);

figure

subplot(3,1,1)
hold on;
plot(t, dps.x1_star, line_style, 'LineWidth', 2)
line([0 t(end)], [min(dps.X1) min(dps.X1)],'LineStyle','--', 'Color', 'red')
line([0 t(end)], [max(dps.X1) max(dps.X1)],'LineStyle','--', 'Color', 'red')
xlabel('Time', 'Interpreter','latex')
ylabel('$x_1$', 'Interpreter','latex')
xlim([0 t(end)])

subplot(3,1,2)
hold on;
plot(t, dps.x2_star, line_style, 'LineWidth', 2)
line([0 t(end)], [min(dps.X2) min(dps.X2)],'LineStyle','--', 'Color', 'red')
line([0 t(end)], [max(dps.X2) max(dps.X2)],'LineStyle','--', 'Color', 'red')
xlabel('Time', 'Interpreter','latex')
ylabel('$x_2$', 'Interpreter','latex')
xlim([0 t(end)])

subplot(3,1,3)
hold on;
plot(t, dps.u_star, line_style, 'LineWidth', 2)
line([0 t(end)], [min(dps.U) min(dps.U)],'LineStyle','--', 'Color', 'red')
line([0 t(end)], [max(dps.U) max(dps.U)],'LineStyle','--', 'Color', 'red')
xlabel('Time', 'Interpreter','latex')
ylabel('$u(k)$', 'Interpreter','latex')
xlim([0 t(end)])

end