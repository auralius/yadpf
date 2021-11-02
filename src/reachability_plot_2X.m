% Draw reachability plot for a system with one state variable
%
% function reachability_plot_2X(dps, ...
%                               terminal_tol)
%
% Arguments:
%
%     dps            = data structure from the dynamic programming solver
%     terminal_state = desired terminal state
%     terminal_tol   = terminal node tollerance
%
function reachability_plot_2X(dps, ...
                              terminal_state, ... 
                              terminal_tol)
figure;
hold on;

x1_star = zeros(1, dps.n_horizon);
x2_star = zeros(1, dps.n_horizon);
buffer_x1  = zeros(dps.nX, dps.n_horizon);
buffer_x2  = zeros(dps.nX, dps.n_horizon);

% Test for all nodes at stage-1 (every possibe ICs)
fprintf('Generating the reachability plot...\n')
fprintf('Progress: ')
ll = 0;

for j = 1:dps.nX
    fprintf(repmat('\b',1,ll));
    ll = fprintf('%.1f %%',j/dps.nX*100);
       
    id = j;
    
    for k = 1 : dps.n_horizon-1
        [r,c] = ind2sub([dps.nX1 dps.nX2], id);
        x1_star(k) = dps.X1(r);
        x2_star(k) = dps.X2(c);
        id = dps.descendant_matrix(id,k);
    end
    
    % The last stage
    [r,c] = ind2sub([dps.nX1 dps.nX2], id);
    x1_star(dps.n_horizon) = dps.X1(r);
    x2_star(dps.n_horizon) = dps.X2(c);
    
    % Check the terminal stage, does it end at the desired terminal node?
    x = [dps.X1(r) dps.X2(c)];
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

% Cinvert to 3D pointclouds
k = repmat(1:dps.n_horizon, size(buffer_x1,1),1);
X = [reshape(buffer_x1,[],1) reshape(buffer_x2,[],1) reshape(k,[],1)];
X = unique(X,'rows');
clear a buffer_x1 buffer_x2;

% Scatter plot the pointclouds
fscatter3(X(:,1), X(:,2), X(:,3), X(:,3), jet);
  
% Make it beautiful
xlim([min(dps.X1) max(dps.X1)]);
ylim([min(dps.X2) max(dps.X2)]);

zlabel(['Stage-' '$k$'], 'Interpreter','latex')
xlabel('$x_1(k)$', 'Interpreter','latex')
ylabel('$x_2(k)$', 'Interpreter','latex')
title('Backward Reachability');

view([-1 -1 1]);

get(gca,'fontname');
set(gca,'fontname','times');  % Set it to times   
end