% The prototype of this function is exactly identical to the standard 
% sub2ind for 3D matrix
%
% References:
% http://tipstrickshowtos.blogspot.com/2010/02/fast-replacement-for-sub2ind.html

function ind = fastsub2ind3(sizes, rows, cols, pages)
ind = rows + (cols-1)*sizes(1) + (pages-1)*sizes(1) *sizes(2);
end
