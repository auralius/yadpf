function V = fastreprowvec(v, n)
% This function duplicates v n-time row-wise, similar to the repmat command 

%------------- BEGIN CODE --------------
V = bsxfun(@times, v, ones(n,length(v)));

end
%------------- END OF CODE --------------