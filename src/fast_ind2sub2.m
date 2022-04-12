function [r, c] = fast_ind2sub2(sz, idx)
% Similar to ind2sub with length(sz) = 2

%------------- BEGIN CODE --------------

r = rem(idx - 1, sz(1)) + 1;
c = (idx - r) / sz(1) + 1;

end
%------------- END OF CODE --------------
