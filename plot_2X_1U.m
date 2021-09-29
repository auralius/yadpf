function plot_2X_1U(dps, line_style)
figure
subplot(3,1,1)
plot(dps.x1_star, line_style)
xlabel('Stage')
ylabel('$x_1$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(3,1,2)
plot(dps.x2_star, line_style)
xlabel('Stage')
ylabel('$x_2$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(3,1,3)
plot(dps.u_star, line_style)
xlabel('Stage')
ylabel('$u$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

end