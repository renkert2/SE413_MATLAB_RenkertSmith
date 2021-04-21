individualSweeps = struct();

ov = o.OptiVars;

for v = ov'
    [X,ft,X_opt,ft_opt,I,PD,DD] = sweep(o, v, 100);
    s = struct();
    s.X = X;
    s.ft = ft;
    s.X_opt = X_opt;
    s.ft_opt = ft_opt;
    s.I = I;
    s.PD = PD;
    s.DD = DD;
    individualSweeps.(v.Sym) = s;
end

save('individualSweeps.mat', 'individualSweeps');

