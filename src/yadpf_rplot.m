function yadpf_rplot(dpf, xf, tol)
% Draw the backward reachability plot for a system of one state and two states
%
% Syntax:  dpf = yadpf_rplot(dpf, xf, tol)
%
% Inputs:
%    dpf - The data structure for the optimal control problem
%    xf  - The terminal state (1 x n vector)
%    tol - The euclidean tollerance to the targeetted terminal state
%
% Outputs: -   
%
% Other m-files required: 
%   reachability_plot_1X.m 
%   reachability_plot_2X.m 
%
% Author:
%   Auralius Manurung
%   Universitas Pertamina 
%   auralius.manurung@ieee.org

%------------- BEGIN CODE --------------

if dpf.n_states > 2
    error('Reachability plot only work for system witn one state and two states!');
end

if dpf.n_states == 1
    reachability_plot_1X(dpf, xf, tol);
elseif dpf.n_states == 2
    reachability_plot_2X(dpf, xf, tol);
end

end
%------------- END OF CODE --------------
