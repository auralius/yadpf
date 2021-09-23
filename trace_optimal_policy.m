function [x1_star, x2_star, u_star] = trace_optimal_policy(decendent_matrix, U_star_matrix, X1, X2, nX1, nX2)
nHorizon = size(decendent_matrix, 2);

x1_star = zeros(nHorizon+1,1);
x2_star = zeros(nHorizon+1,1);
u_star = zeros(nHorizon+1,1);

id = 1;
for k = 1 : nHorizon      
    [xid,vid] = ind2sub([nX1 nX2], id);
    
    % Convert index back to actual value
    x1 = X1(xid);
    x2 = X2(vid);
            
    x1_star(k) = x1;
    x2_star(k) = x2;
    u_star(k) = U_star_matrix(id, k);
    
    id = decendent_matrix(id,k);
end

[xid,vid] = ind2sub([nX1 nX2], id);
    
% Convert index back to actual value
x1 = X1(xid);
x2 = X2(vid);

x1_star(end) = x1;
x2_star(end) = x2;
end