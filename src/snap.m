% Convert a given value to closest integer ranging from 1 to N
%
% min_val will be converted to 1 and max_val will be converted to N.
%

function snapped_val = snap(val, min_val, max_val, N)

snapped_val = floor(((val - min_val) ./ (max_val - min_val)  .* (N-1))+1);
snapped_val = max(min(snapped_val, N), 1);

end