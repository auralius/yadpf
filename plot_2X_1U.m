function plot_2X_1U(dps)
figure
subplot(3,1,1)
plot(dps.x1_star, '-*')
xlabel('Stage')
ylabel('$x_1$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(3,1,2)
plot(dps.x2_star, '-*')
xlabel('Stage')
ylabel('$x_2$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(3,1,3)
plot(dps.u_star, '-*')
xlabel('Stage')
ylabel('$u$', 'Interpreter','latex')
xlim([1 dps.n_horizon])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

end