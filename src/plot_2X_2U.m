function plot_2X_2U(dps, line_style)
figure
subplot(4,1,1)
plot(dps.x1_star, line_style, 'LineWidth', 2)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_1(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(4,1,2)
plot(dps.x2_star, line_style, 'LineWidth', 2)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_2(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(4,1,3)
plot(dps.u1_star, line_style, 'LineWidth', 2)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$u_1(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon])
ylim([min(dps.U1), max(dps.U1)])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(4,1,4)
plot(dps.u2_star, line_style, 'LineWidth', 2)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$u_2(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon])
ylim([min(dps.U2), max(dps.U2)])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

end