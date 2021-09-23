%%
function x = create_new_state(name, lb, ub, step)

x.ub = ub;
x.lb = lb;
x.step = step;
x.elements = lb:step:ub;
x.name = name;
x.n = length(x.elements);

end
