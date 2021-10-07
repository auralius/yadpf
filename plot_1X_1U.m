function plot_1X_1U(dps)

figure
subplot(2,1,1)
bar(dps.x_star)
xlabel('Stage')
ylabel('$x$', 'Interpreter','latex')
xlim([1 dps.n_horizon+1])

ax = gca;
ax.XTick = unique(round(ax.XTick) );

subplot(2,1,2)
bar(dps.u_star)
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$u(k)$', 'Interpreter','latex')
xlim([1 dps.n_horizon+1])

ax = gca;
ax.XTick = unique(round(ax.XTick) );
end