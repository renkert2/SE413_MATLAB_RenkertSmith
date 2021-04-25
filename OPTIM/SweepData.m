classdef SweepData
    %SWEEPOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Vars optiVar % Design Variables being swept
        N_vars double % number of design variables
        X
        F % Primary objective function value
        
        X_opt
        F_opt
        
        I
        PD % Performance Data
        DD % Design Data
        OO OptimOutput % Optimization Output
    end
    
    methods
        function plotObjective(obj)
            switch obj.N_vars
                case 1
                    figure
                    plot(obj.X,obj.F)
                    hold on
                    plot(obj.X_opt, obj.F_opt, '.r', 'MarkerSize', 20)
                    hold off
                case 2
                    figure
                    surf(obj.X(:,:,1), obj.X(:,:,2),obj.F);
                    hold on
                    plot3(obj.X_opt(1), obj.X_opt(2), obj.F_opt, '.r', 'MarkerSize', 20);
                    hold off
            end
            
            title("Sweep Plot")
            xlabel(latex(obj.Vars(1)), 'Interpreter', 'latex')
            if obj.N_vars == 2
                ylabel(latex(obj.Vars(2)), 'Interpreter', 'latex')
                zlabel('Objective', 'Interpreter', 'latex')
            else
                ylabel('Objective', 'Interpreter', 'latex')
            end
        end
        
        function plotGeneral(obj, y, y_range)
            x = obj.X;
            plot(x,y);
            xlim([obj.Vars.lb obj.Vars.ub]);
            ylim(y_range);
            
            % NaN Highlighting
            nan_i = find([isnan(y)]);
            
            if nan_i
                split_i = find(diff(nan_i) > 1) + 1;
                nan_groups = [1 split_i numel(y)];
                hold on
                for i = 1:(numel(split_i)+1)
                    group_i = nan_i(nan_i >= nan_groups(i) & nan_i <= nan_groups(i+1));
                    start_x = x(max(1,group_i(1)-1));
                    finish_x = x(min(group_i(end)+1, numel(x)));
                    
                    patch([start_x start_x finish_x finish_x], [y_range fliplr(y_range)], 'r','FaceAlpha',0.2,'EdgeColor','none');
                end
                hold off
            end
            
            % Constraint Highlighting
        end
        
        function corrplot(obj_array)
            N = numel(obj_array);
            t = tiledlayout(N, N);
            title(t,"Design Variable Coupling");
            t.TileSpacing = 'tight';
            t.Padding = 'compact';
            xlabel(t,"Independent Variable");
            ylabel(t, "Dependent Variable");
            lblspecs = {'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold'};
            
            vars = [obj_array.Vars];
            lb = [vars.lb];
            ub = [vars.ub];
            
            tile_counter = 1;
            CMat = zeros(N,N);
            for i = 1:N
                for j = 1:N
                    nexttile(tile_counter)
                    [CMat(j,i),~,y] = correlate(obj_array, j, i);
                    tile_counter = tile_counter+1;
                    
                    y_range = [lb(i) ub(i)];
                    plotGeneral(obj_array(j), y, y_range);
                    
                    if j == 1
                        ylabel(latex(vars(i)),lblspecs{:});
                    else
                        set(gca,'Yticklabel',[])
                    end
                    if i == numel(vars)
                        xlabel(latex(vars(j)), lblspecs{:});
                    else
                        set(gca,'Xticklabel',[])
                    end
                end
            end
        end
        
        function [c,x,y] = correlate(obj_array, var1, var2)
            x = obj_array(var1).X;
            dd_array = [obj_array(var1).DD.(obj_array(var2).Vars.Parent.Name)];
            data = [dd_array.(obj_array(var2).Vars.Sym)];
            y = SweepData.processSweep(obj_array(var1).I, data, length(x));
            m = [x',y'];
            m = m(all(~isnan(m),2),:);
            c = corrcoef(m);
            c = c(1,2);
        end
        
    end
    
    methods (Static)
        function y = processSweep(I,data,len)
            y = NaN(1,len);
            
            for i = 1:numel(data)
                index = I(i);
                y(index) = data(i);
            end
        end
    end
end

