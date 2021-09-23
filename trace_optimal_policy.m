function [x1_star, x2_star, u_star] = trace_optimal_policy(dps)

x1_star = zeros(dps.n_horizon+1,1);
x2_star = zeros(dps.n_horizon+1,1);
u_star = zeros(dps.n_horizon+1,1);

id = 1;
for k = 1 : dps.n_horizon      
    [xid,vid] = ind2sub(dps.states_sizes, id);
    
    % Convert index back to actual value
    x1 = dps.states(1).elements(xid);
    x2 = dps.states(2).elements(vid);
            
    x1_star(k) = x1;
    x2_star(k) = x2;
    u_star(k) = dps.U_star_matrix(id, k);
    
    id = dps.decendent_matrix(id,k);
end

[xid,vid] = ind2sub(dps.states_sizes, id);
    
% Convert index back to actual value
x1 = dps.states(1).elements(xid);
x2 = dps.states(2).elements(vid);

x1_star(end) = x1;
x2_star(end) = x2;

end