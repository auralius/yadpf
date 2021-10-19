% Draw reachability plot for a system with one state variable
%
% function reachability_plot_1X(dps, ...
%                               terminal_tol)
%
% Arguments:
%
%     dps            = data structure from the dynamic programming solver
%     terminal_state = desired terminal state
%     terminal_tol   = terminal node tollerance
%
function reachability_plot_1X(dps, ...
                              terminal_state, ... 
                              terminal_tol)
figure;
hold on;

x_star = zeros(1, dps.n_horizon);
buffer = zeros(dps.nX, dps.n_horizon);

% Test for all nodes at stage-1 (every possibe ICs)
fprintf('Generating the reachability plot...\n')
fprintf('Progress: ')
ll = 0;

for j = 1:dps.nX
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',j/dps.nX*100);
       
    id = j;
    
    for k = 1 : dps.n_horizon-1
        x_star(k) = dps.X(id);
        id = dps.descendant_matrix(id,k);
    end
    
    % The last stage
    x_star(dps.n_horizon) = dps.X(id);
    
    % Check the terminal stage, does it end at the desired terminal node?
    if abs(dps.X(id) - terminal_state) < terminal_tol
        buffer(j,:) = x_star;     % If yes, keep them
    end
end

fprintf('\nComplete!\n')

% Plot only the maximums and the minimums, color the area in between.
buffer(~any(buffer,2),:) = [];  % Delete rows that are all zeros
mins = min(buffer);
maxs = max(buffer);
k = 1 : dps.n_horizon;
plot(k, mins);
plot(k, maxs);
patch([k fliplr(k)], [mins fliplr(maxs)], 'g')

xlim([1 dps.n_horizon+1])
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_1(k)$', 'Interpreter','latex')

ax = gca;
ax.XTick = unique(round(ax.XTick) );
end