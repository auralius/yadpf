function plot_1X_2U(dps, line_style)
n =  length(dps.u1_star);
tf = length(dps.u1_star) * dps.T_dyn;
t = linspace(0, tf, n);

figure

subplot(3,1,1)
hold on;
plot(t, dps.x_star, line_style, 'LineWidth', 2)
line([0 t(end)], [min(dps.X) min(dps.X)],'LineStyle','--', 'Color', 'red')
line([0 t(end)], [max(dps.X) max(dps.X)],'LineStyle','--', 'Color', 'red')
xlabel('Time', 'Interpreter','latex')
ylabel('$x$', 'Interpreter','latex')
xlim([0 t(end)])

subplot(3,1,2)
hold on;
plot(t, dps.u1_star, line_style, 'LineWidth', 2)
line([0 t(end)], [min(dps.U1) min(dps.U1)],'LineStyle','--', 'Color', 'red')
line([0 t(end)], [max(dps.U1) max(dps.U1)],'LineStyle','--', 'Color', 'red')
xlabel('Time', 'Interpreter','latex')
ylabel('$u_1(k)$', 'Interpreter','latex')
xlim([0 t(end)])

subplot(3,1,3)
hold on;
plot(t, dps.u2_star, line_style, 'LineWidth', 2)
line([0 t(end)], [min(dps.U2) min(dps.U2)],'LineStyle','--', 'Color', 'red')
line([0 t(end)], [max(dps.U2) max(dps.U2)],'LineStyle','--', 'Color', 'red')
xlabel('Time', 'Interpreter','latex')
ylabel('$u_2(k)$', 'Interpreter','latex')
xlim([0 t(end)])

end