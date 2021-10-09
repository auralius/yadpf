function reachability_plot_1X_1U(dps, terminal_tol)
figure;
hold on;

% Unpack the data, field access is slow
nX                = length(dps.X);
n_horizon         = dps.n_horizon;
X                 = dps.X;
descendant_matrix = dps.descendant_matrix;
J                 = dps.J;

x_star = zeros(n_horizon, 2);
X_buffer = [];

[~,id_target]  = min(J(:,n_horizon));

% Test for all nodes at stage-1 (every possibe ICs)
fprintf('Generating the reachability plot...\n')
fprintf('Progress: ')
ll = 0;

for j = 1:nX
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',j/nX*100);
       
    id = j;
    
    for k = 1 : n_horizon-1
        x_star(k,:) = [k X(id)];
        id = descendant_matrix(id,k);
    end
    
    % The last stage
    x_star(n_horizon,:) = [n_horizon X(id)];
    
    % Check the terminal stage, does it end at the desired terminal node?
    if abs(id - id_target) < terminal_tol
        X_buffer = [X_buffer; x_star];  % If yes, keep them
    end
end

X_buffer = unique(X_buffer,'rows');

fprintf('\nComplete!\n')

scatter(X_buffer(:,1), X_buffer(:,2), 'Marker', '.');

xlim([1 dps.n_horizon+1])
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_1(k)$', 'Interpreter','latex')

ax = gca;
ax.XTick = unique(round(ax.XTick) );
end