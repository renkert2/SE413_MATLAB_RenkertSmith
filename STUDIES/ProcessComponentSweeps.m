load battSweep
load motorSweep
load propSweep

%%


% hold on
% plot3(X_opt(1), X_opt(2), ft_opt, '.r', 'MarkerSize', 20);
% hold off
%% Thrust Ratio
tr = getGrid(battSweep.I,  [battSweep.PD.ThrustRatio], size(battSweep.ft));
figure
surf(battSweep.X(:,:,1), battSweep.X(:,:,2),tr);
%%
title("Carpet Plot")
xlabel(latex(vars(1)), 'Interpreter', 'latex')
if N_vars == 2
    ylabel(latex(vars(2)), 'Interpreter', 'latex')
    zlabel('Flight Time (s)', 'Interpreter', 'latex')
else
    ylabel('Flight Time (s)', 'Interpreter', 'latex')
end

function grid = getGrid(I,data,size)
    grid = NaN(size);
    
    for i = 1:numel(data)
        index = I(:,i);
        grid(index(1), index(2)) = data(i);
    end
end