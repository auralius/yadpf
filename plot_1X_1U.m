function plot_1X_1U(dps)

figure
subplot(2,1,1)
plot(dps.x_star, '-*')
xlabel('Stage')
ylabel('$x$', 'Interpreter','latex')
xlim([1 dps.n_horizon+1])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(2,1,2)
plot(dps.u_star, '-*')
xlabel('Stage')
ylabel('$u$', 'Interpreter','latex')
xlim([1 dps.n_horizon+1])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

end