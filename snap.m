%%
% This function will convert a given value to closest integer ranging
% from 1 to N+1.
% minVal will be converted to 1 and maxVal will be converted to N+1.

function snapped_val = snap(val, min_val, max_val, N)

range = max_val - min_val;
snapped_val = round((val - min_val) / range * N + 1);

end