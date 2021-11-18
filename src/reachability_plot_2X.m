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

x1_star = zeros(1, dpf.n_horizon);
x2_star = zeros(1, dpf.n_horizon);
buffer_x1  = zeros(dpf.nXX, dpf.n_horizon);
buffer_x2  = zeros(dpf.nXX, dpf.n_horizon);

% Test for all nodes at stage-1 (every possibe ICs)
fprintf('Generating the reachability plot...\n')
fprintf('Progress: ')
ll = 0;

for j = 1:dpf.nXX
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',j/dpf.nXX*100);
       
    id = j;
    
    for k = 1 : dpf.n_horizon-1
        [r,c] = ind2sub([dpf.nX(1)], id);
        x1_star(k) = dpf.states{1}(r);
        x2_star(k) = dpf.states{2}(c);
        id = dpf.descendant_matrix(k,id);
    end
    
    % The last stage
    [r,c] = ind2sub([dpf.nX(1) dpf.nX(2)], id);
    x1_star(dpf.n_horizon) = dpf.states{1}(r);
    x2_star(dpf.n_horizon) = dpf.states{2}(c);
    
    % Check the terminal stage, does it end at the desired terminal node?
    x = [dpf.states{1}(r) dpf.states{2}(c)];
    if norm(x - terminal_state) < terminal_tol
        buffer_x1(j,:) = x1_star;     % If yes, keep them
        buffer_x2(j,:) = x2_star;     % If yes, keep them
    end
end
clear x1_star x2_star;

fprintf('\nComplete!\n')

% Delete rows that all zeros
a = any(buffer_x1+buffer_x2, 2);
buffer_x1(~a,:) = [];  
buffer_x2(~a,:) = [];  

% Convert to 3D pointclouds
k = repmat(1:dpf.n_horizon, size(buffer_x1,1),1);
X = [reshape(buffer_x1,[],1) reshape(buffer_x2,[],1) reshape(k,[],1)];
X = unique(X,'rows');
clear a buffer_x1 buffer_x2;

if isempty(X)
    error('No reachable states are foud, increase the tollerance...\n');
end

% Scatter plot the pointclouds
fscatter3(X(:,1), X(:,2), X(:,3), X(:,3), jet);
  
% Make it beautiful
xlim([min(dpf.states{1}) max(dpf.states{1})]);
ylim([min(dpf.states{2}) max(dpf.states{2})]);

zlabel(['Stage-' '$k$'], 'Interpreter','latex')
xlabel('$x_1(k)$', 'Interpreter','latex')
ylabel('$x_2(k)$', 'Interpreter','latex')
title('Backward Reachability');

view([-1 -1 1]);

get(gca,'fontname');
set(gca,'fontname','times');  % Set it to times   
end
%------------- END OF CODE --------------
