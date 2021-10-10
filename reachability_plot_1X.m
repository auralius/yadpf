% Draw reachability plot for a system with one state variable
%
% function reachability_plot_1X(dps, ...
%                               terminal_tol)
%
% Arguments:
%
%     dps          = data structure from the dynamic programming solver
%     terminal_tol = terminal node tollerance (integer value)
%
function reachability_plot_1X(dps, ...
                              terminal_tol)
figure;
hold on;

% Unpack the data, field access is slow
nX                = length(dps.X);
n_horizon         = dps.n_horizon;
X                 = dps.X;
descendant_matrix = dps.descendant_matrix;
J                 = dps.J;

x_star = zeros(1, n_horizon);
buffer = zeros(nX,n_horizon);

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
        x_star(k) = X(id);
        id = descendant_matrix(id,k);
    end
    
    % The last stage
    x_star(n_horizon) = X(id);
    
    % Check the terminal stage, does it end at the desired terminal node?
    if abs(id - id_target) < terminal_tol
        buffer(j,:) = x_star;     % If yes, keep them
    end
end

fprintf('\nComplete!\n')

buffer(~any(buffer,2),:) = [];  % Delete rows that are all zeros
mins = min(buffer);
maxs = max(buffer);
k = 1:n_horizon;
plot(k,mins);
plot(k,maxs);
patch([k fliplr(k)], [mins fliplr(maxs)], 'g')


xlim([1 dps.n_horizon+1])
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_1(k)$', 'Interpreter','latex')

ax = gca;
ax.XTick = unique(round(ax.XTick) );
end