function plot_1X_1U(dps, line_style)

figure
subplot(2,1,1)
plot(dps.x_star, line_style, 'LineWidth', 2)
xlabel('Stage')
ylabel('$x$', 'Interpreter','latex')
xlim([1 dps.n_horizon+1])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(2,1,2)
plot(dps.u_star, line_style, 'LineWidth', 2)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$u(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon+1])

ax = gca;
ax.XTick = unique(round(ax.XTick) );
end