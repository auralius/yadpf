function u = create_new_input(name, lb, ub, step)

u.ub = ub;
u.lb = lb;
u.step = step;
u.elements = lb:step:ub;
u.name = name;
u.n = length(u.elements);

end
