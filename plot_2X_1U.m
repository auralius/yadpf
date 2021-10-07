function plot_2X_1U(dps, line_style)
figure
subplot(3,1,1)
plot(dps.x1_star, line_style)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_1(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(3,1,2)
plot(dps.x2_star, line_style)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_2(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(3,1,3)
plot(dps.u_star, line_style)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$u(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

end