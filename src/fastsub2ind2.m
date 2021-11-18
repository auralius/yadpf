function ind = fastsub2ind2(sizes, rows, cols)
% The prototype of this function is exactly identical to the standard 
% sub2ind for 2D matrix
%
% References:
% http://tipstrickshowtos.blogspot.com/2010/02/fast-replacement-for-sub2ind.html

%------------- BEGIN CODE --------------

ind = rows + (cols-1)*sizes(1);

end
%------------- END OF CODE --------------
