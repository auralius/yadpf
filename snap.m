% Convert a given value to closest integer ranging from 1 to N+1
%
% min_val will be converted to 1 and max_val will be converted to N+1.
%

function snapped_val = snap(val, min_val, max_val, N)

snapped_val = round((val - min_val) ./ (max_val - min_val) .* N + 1);

end