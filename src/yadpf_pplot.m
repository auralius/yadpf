function yadpf_pplot(dpf)

if strcmp(dpf.method, 'vi') == 0
    error('yadpf_pplot is only for value iteration method!');
end

if dpf.n_states > 2
    error('yadpf_pplot only works for a system with 1 state and 2 states!')
end

if dpf.n_states == 1
    for i = 1 : dpf.n_inputs
        figure;
        area(dpf.states{1}, dpf.inputs{i}(dpf.U_star_matrix{i}), ...
            'LineWidth', 2);
        xlabel('x', 'Interpreter','tex');
        ylabel(['u_' num2str(i) '$'], 'Interpreter','tex');
    end

elseif dpf.n_states == 2
    policy_matrix = zeros(dpf.nX(2), dpf.nX(1));
    for i = 1 : dpf.n_inputs
        figure;
        hold on

        for k = 1 : dpf.nXX
            [r,c] = ind2sub(dpf.nX, k);
            policy_matrix(c,r) = dpf.inputs{i}(dpf.U_star_matrix{i}(k));
        end

        mesh(dpf.states{2}, dpf.states{1}, policy_matrix');
        plot3(dpf.x_star_unsimulated{2}(1:end-1), ...
              dpf.x_star_unsimulated{1}(1:end-1), ...
              dpf.u_star_unsimulated{1}(1:end), ...
              'r', 'LineWidth', 2);
        colorbar;

        xlabel('x_2', 'Interpreter', 'tex');
        ylabel('x_1', 'Interpreter', 'tex');
        zlabel(['u_' num2str(i)], 'Interpreter', 'tex');

        l = ['Policy matrix for u_' num2str(1)];
        title(l, 'Interpreter','tex');

        view([1 1 1])

        ax = gca;
        set(ax, "SortMethod", 'childorder');
    end
end

end
