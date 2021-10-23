function dps = forward_trace(dps, ic)

if (dps.n_states == 1 && dps.n_inputs == 1)
    dps = trace_1X_1U(dps, ic);
elseif (dps.n_states == 1 && dps.n_inputs == 2)
    dps = trace_1X_2U(dps, ic);    
elseif (dps.n_states == 2 && dps.n_inputs == 1)    
    dps = trace_2X_1U(dps, ic(1), ic(2));
elseif (dps.n_states == 2 && dps.n_inputs == 2)    
    dps = trace_2X_2U(dps, ic(1), ic(2));
elseif (dps.n_states == 3 && dps.n_inputs == 1)    
    dps = trace_3X_1U(dps, ic(1), ic(2), ic(3));
elseif (dps.n_states == 3 && dps.n_inputs == 2)    
    dps = trace_3X_2U(dps, ic(1), ic(2), ic(3));
end


end