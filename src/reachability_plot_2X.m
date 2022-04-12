function reachability_plot_2X(dpf, terminal_state, terminal_tol)
% Draw reachability plot for a system with one state variable
%
% Inputs:
%
%     dpf - data structure from the dynamic programming function
%     terminal_state - desired terminal state
%     terminal_tol - terminal node tollerance
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
x_star_unsimulated = dpf.x_star_unsimulated;

clear dpf;

x1 = zeros(1, n_horizon);
x2 = zeros(1, n_horizon);

x1s  = zeros(nXX, n_horizon);
x2s  = zeros(nXX, n_horizon);

% Test for all nodes at stage-1 (every possibe ICs)
fprintf('Generating the reachability plot...\n')
fprintf('Progress: ')
ll = 0;

step = max(floor(nXX/10), 1);
for j = 1 : nXX    
    if (rem(j-1, step) == 0)
        fprintf(repmat('\b', 1, ll));
        ll = fprintf('%.1f %%',(j-1) / nXX * 100);
    end
       
    id = j;
    
    for k = 1 : n_horizon-1
        [r, c] = fast_ind2sub2([nX(1) nX(2)], id);    
        x1(k) = states{1}(r);
        x2(k) = states{2}(c);
        id = descendant_matrix(k, id);
    end
    
    % The last stage
    [r, c] = fast_ind2sub2([nX(1) nX(2)], id);
    x1(n_horizon) = states{1}(r);
    x2(n_horizon) = states{2}(c);
    
    % Check the terminal stage, does it end at the desired terminal node?
    x3s = [states{1}(r) states{2}(c)];
    if norm(x3s - terminal_state) < terminal_tol
        x1s(j,:) = x1;     % If yes, keep them
        x2s(j,:) = x2;     % If yes, keep them
    end
end

clear x1 x2 descendant_matrix;

fprintf('\nComplete!\n')

% Delete rows that all zeros
a = any(x1s + x2s, 2);
x1s(~a,:) = [];  
x2s(~a,:) = [];  

% Convert to 3D pointclouds
k = repmat(1 : n_horizon, size(x1s,1),1);
X = [reshape(x1s, [], 1) reshape(x2s, [], 1) reshape(k, [], 1)];
X = unique(X,'rows');
clear a x1s x2s;

if isempty(X)
    error('No reachable states are foud, increase the tollerance...\n');
end

% Scatter plot the pointclouds
fscatter3(X(:,1), X(:,2), X(:,3), X(:,3), jet);
hold on
plot3(x_star_unsimulated{1}, x_star_unsimulated{2}, ...
      1:n_horizon, 'k', 'LineWidth', 3);
  
% Make it beautiful
xlim([min(states{1}) max(states{1})]);
ylim([min(states{2}) max(states{2})]);

zlabel(['Stage-' '$k$'], 'Interpreter','latex')
xlabel('$x_1(k)$', 'Interpreter','latex')
ylabel('$x_2(k)$', 'Interpreter','latex')
title('Backward Reachability');

view([-1 -1 1]);

ax = gca;
ax.SortMethod = 'childorder';
ax.FontName = 'Times';
end
%------------- END OF CODE --------------
