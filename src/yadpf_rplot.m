function yadpf_rplot(dpf, xf, tol)

if dpf.n_states > 2
    error('Reachability plot only work for system witn one state and two states!');
end

if dpf.n_states == 1
    reachability_plot_1X(dpf, xf, tol);
elseif dpf.n_states == 2
    reachability_plot_2X(dpf, xf, tol);
end

end