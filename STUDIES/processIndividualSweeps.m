global individualSweeps
load individualSweeps.mat


%%
% plotSweep("N_s", [individualSweeps.N_s.DD.ThrustRatio])
% vars = ["D", "P", "N_p", "N_s", "kV", "Rm"];
% comps = ["Propeller", "Propeller", "Battery", "Battery", "Motor", "Motor"];

correlate("D", "Propeller", "D", true);

N = numel(vars);
CMat = zeros(N,N);
for i = 1:N
    for j = 1:N
        CMat(i,j) = correlate(vars(i), comps(j), vars(j), false);
    end
end

function [c,m] = correlate(var1, comp2, var2, figure_flag)
    global individualSweeps
    x = individualSweeps.(var1).X;
    dd_array = [individualSweeps.(var1).DD.(comp2)];
    if var2 == "kV"
        data = [dd_array.("K_t")];
        data = PMSMMotor.KtTokV(data);
    else
        data = [dd_array.(var2)];
    end
    
    y = processSweep(individualSweeps.(var1).I, data, length(x));
    
    m = [x',y'];
    m = m(all(~isnan(m),2),:);
    c = corrcoef(m);
    c = c(1,2);
    
    if figure_flag
        figure
        plot(x,y);
        xlabel("$$"+var1+"$$", 'Interpreter', 'latex');
        ylabel("$$"+var2+"$$", 'Interpreter', 'latex');
    end
end

function plotSweep(var, data)
    global individualSweeps
    v = individualSweeps.(var);
    x = v.X;
    y = processSweep(individualSweeps.(var).I, data, length(x));
    
    figure
    plot(x,y);
    xlabel("$$"+var+"$$", 'Interpreter', 'latex');
end

function y = processSweep(I,data, len)
    y = NaN(1,len);
    
    for i = 1:numel(data)
        index = I(i);
        y(index) = data(i);
    end
end