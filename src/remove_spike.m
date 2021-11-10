% Remove spike whose width is n samples
% There must be at least 100 samples of data points

function x = remove_spike(x,n)
if length(x) > 100 % minimum 100 samples
    [~,loc] = findpeaks(x, 'Threshold', n);
    if ~isempty(loc)
        x(loc) = NaN;
        x = fillmissing(x,'previous');
    end
end
end