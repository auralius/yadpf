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

% Structure is slow, so we will unload the strucutre
n_horizon = dpf.n_horizon;
nXX = dpf.nXX;
nX = dpf.nX;
states = dpf.states;
descendant_matrix = dpf.descendant_matrix;

clear dpf;

x_star = zeros(n_horizon, nX);
buffer = zeros(n_horizon, nX);

% Test for all nodes at stage-1 (every possibe ICs)
fprintf('Generating the reachability plot...\n')

id = 1 : nXX;

for k = 1 : n_horizon-1
    x_star(k, :) = states{1}(id);
    id = descendant_matrix(k,id);
end

% The last stage
x_star(n_horizon, :) = states{1}(id);

% Check the terminal stage, does it end at the desired terminal node?
for j = 1 : nXX
    if abs(states{1}(id(j)) - terminal_state) < terminal_tol
        buffer(:, j) = x_star(:, j);     % If yes, keep them
    end
end

fprintf('Complete!\n')

buffer = transpose(buffer);

% Plot only the maximums and the minimums, color the area in between.
buffer(~any(buffer, 2), :) = [];  % Delete rows that are all zeros

if isempty(buffer)
    error('No reachable states are foud, increase the tollerance...\n');
end

mins = min(buffer);
maxs = max(buffer);
k = 1 : n_horizon;
plot(k, mins);
plot(k, maxs);
patch([k fliplr(k)], [mins fliplr(maxs)], 'g')

xlim([1 n_horizon+1])
xlabel(['Stage-' 'k'], 'Interpreter','tex')
ylabel('x_1(k)', 'Interpreter','tex')
title('Backward Reachability Plot');

ax = gca;
set(ax, "XTick", unique(round(get(ax,"XTick"))));

get(gca,'fontname');
set(gca,'fontname','times')  % Set it to times
end
%------------- END OF CODE --------------
