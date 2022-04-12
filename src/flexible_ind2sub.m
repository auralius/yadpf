function b = flexible_ind2sub(sz, ind)
% The output for [s1, s2, ...] = ind2sub(sz, ind) is comma separated. 
% In some cases, we do not know yet the length.

%------------- BEGIN CODE --------------

b = cell(1, length(sz));
[b{:}] = ind2sub(sz, ind);

end
%------------- END OF CODE --------------
