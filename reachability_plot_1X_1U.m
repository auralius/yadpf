function reachability_plot_1X_1U(dps, terminal_tol)

figure;
hold on;

x_star = zeros(dps.n_horizon, 1);

% Unpack the data, field access is slow
nX                = length(dps.X);
n_horizon         = dps.n_horizon;
X                 = dps.X;
descendant_matrix = dps.descendant_matrix;
J                 = dps.J;

[~,id_target]  = min(J(:,n_horizon));

% Test for all nodes at stage-1 (every possibe ICs)
for j = 1:nX
    id = j;
    for k = 1 : n_horizon-1
        x_star(k) = X(id);
        id = descendant_matrix(id,k);        
    end    
    % The last stage
    x_star(n_horizon,:) = X(id);       
    
    % Check the terminal stage, does it end at the desired terminal node?
    if abs(id - id_target) < terminal_tol
        plot(x_star, '-k', 'LineWidth', 0.1);          %If yes, plot them
    end
end

xlim([1 dps.n_horizon+1])
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_1(k)$', 'Interpreter','latex')

ax = gca;
ax.XTick = unique(round(ax.XTick) );
end