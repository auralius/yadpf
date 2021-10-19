function V = fastreprowvec(v, n)

V = bsxfun(@times, v, ones(n,length(v)));

end