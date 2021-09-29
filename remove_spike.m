function x = remove_spike(x)
if length(x) > 10
    [~,loc] = findpeaks(x, 'Threshold',1);
    if ~isempty(loc)
        x(loc) = NaN;
        x = fillmissing(x,'previous');
    end
end
end