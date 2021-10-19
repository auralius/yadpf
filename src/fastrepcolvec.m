function V = fastrepcolvec(v, n)

V = bsxfun(@times, v, ones(length(v),n));

end