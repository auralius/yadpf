% Remove spike whose width is only one stage

function x = remove_spike(x)
if length(x) > 1000 % at least 10 stages
    [~,loc] = findpeaks(x, 'Threshold',1);
    if ~isempty(loc)
        x(loc) = NaN;
        x = fillmissing(x,'previous');
    end
end
end