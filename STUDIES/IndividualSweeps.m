individualSweeps = struct();

ov = o.OptiVars;

for v = ov'
    if ismember(v.Sym,["D","P","N_s"])
        rev_flag = true;
    else
        rev_flag = false;
    end
    
    [X,ft,X_opt,ft_opt,I,PD,DD] = sweep(o, v, 100, 'ReverseSearch', rev_flag);
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

