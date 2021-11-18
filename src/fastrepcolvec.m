function V = fastrepcolvec(v, n)
% This function duplicates v n-time column-wise, similar to the repmat command 

%------------- BEGIN CODE --------------

V = bsxfun(@times, v, ones(length(v),n));

end

%------------- END OF CODE --------------
