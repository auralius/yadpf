function ind = flexible_sub2ind(sz, varargin)
% sub2ind(sz, sub) does not work if length(sz) == 1

%------------- BEGIN CODE --------------

if length(sz) > 1
    ind = uint32(sub2ind(sz, varargin{:}));
else
    ind = uint32(varargin{:});
end

end
%------------- END OF CODE --------------
