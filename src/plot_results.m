function plot_results(dps, linestyle)

if (dps.n_states == 1 && dps.n_inputs == 1)
    plot_1X_1U(dps, linestyle);
elseif (dps.n_states == 1 && dps.n_inputs == 2)
    plot_1X_2U(dps, linestyle);    
elseif (dps.n_states == 2 && dps.n_inputs == 1)    
    plot_2X_1U(dps, linestyle);
elseif (dps.n_states == 2 && dps.n_inputs == 2)    
    plot_2X_2U(dps, linestyle);
elseif (dps.n_states == 3 && dps.n_inputs == 1)    
    plot_3X_1U(dps, linestyle);
elseif (dps.n_states == 3 && dps.n_inputs == 2)    
    plot_3X_2U(dps, linestyle);
end

get(gca,'fontname'); 
set(gca,'fontname','times')  % Set it to times

end