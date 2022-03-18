function [snapped_val, idx] = snap(val, min_val, max_val, N)
% Convert a given value to closest integer ranging from 1 to N
% If requested, store the indices whose values are outside the boundaries
%
% min_val will be converted to 1 and max_val will be converted to N.
%

%------------- BEGIN CODE --------------

snapped_val = floor(((val - min_val) ./ (max_val - min_val)  .* (N-1))+1);

% Before we apply the boudaries, we store the linear indices of elements
% that are outside the boudaries. This information might be useful!
if nargout >1
    idx = [find(snapped_val > N); find(snapped_val < 1)];    
end

snapped_val = max(min(snapped_val, N), 1);

end
%------------- END OF CODE --------------
