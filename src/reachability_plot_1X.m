function reachability_plot_1X(dpf, terminal_state, terminal_tol)
% Draw reachability plot for a system with one state variable
%
% Inputs:
%
%     dpf -  data structure from the dynamic programming function
%     terminal_state - desired terminal state
%     terminal_tol - terminal node tollerance
%
%
% Author:
%   Auralius Manurung
%   Universitas Pertamina 
%   auralius.manurung@ieee.org

%------------- BEGIN CODE --------------

figure;
hold on;

x_star = zeros(1, dpf.n_horizon);
buffer = zeros(dpf.nX, dpf.n_horizon);

% Test for all nodes at stage-1 (every possibe ICs)
fprintf('Generating the reachability plot...\n')
fprintf('Progress: ')
ll = 0;

for j = 1:dpf.nXX
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',j/dpf.nXX*100);
       
    id = j;
    
    for k = 1 : dpf.n_horizon-1
        x_star(k) = dpf.states{1}(id);
        id = dpf.descendant_matrix(k,id);
    end
    
    % The last stage
    x_star(dpf.n_horizon) = dpf.states{1}(id);
    
    % Check the terminal stage, does it end at the desired terminal node?
    if abs(dpf.states{1}(id) - terminal_state) < terminal_tol
        buffer(j,:) = x_star;     % If yes, keep them
    end
end

fprintf('\nComplete!\n')

% Plot only the maximums and the minimums, color the area in between.
buffer(~any(buffer,2),:) = [];  % Delete rows that are all zeros

if isempty(buffer)
    error('No reachable states are foud, increase the tollerance...\n');
end

mins = min(buffer);
maxs = max(buffer);
k = 1 : dpf.n_horizon;
plot(k, mins);
plot(k, maxs);
patch([k fliplr(k)], [mins fliplr(maxs)], 'g')

xlim([1 dpf.n_horizon+1])
xlabel(['Stage-' '$k$'], 'Interpreter','latex')
ylabel('$x_1(k)$', 'Interpreter','latex')
title('Backward Reachability Plot');

ax = gca;
ax.XTick = unique(round(ax.XTick) );

get(gca,'fontname');  
set(gca,'fontname','times')  % Set it to times
end
%------------- END OF CODE --------------
