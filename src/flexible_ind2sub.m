function b = flexible_ind2sub(sz, ind)
% The output for [s1, s2, ...] = ind2sub(sz, ind) is comma separated. 
% In some cases, we do not know yet the length.

%------------- BEGIN CODE --------------

b = cell(1, length(sz));
if size(sz, 2) == 1
    [b{:}] = ind2sub([1, sz], ind);
else
    [b{:}] = ind2sub(sz, ind);
end

end
%------------- END OF CODE --------------
