%%
% This function will convert a given value to closest integer ranging
% from 1 to N+1.
% minVal will be converted to 1 and maxVal will be converted to N+1.

function snapped_val = snap(val, min_val, max_val, N)

snapped_val = floor((val - min_val) ./ (max_val - min_val) .* N + 1);

end