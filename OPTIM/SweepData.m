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
        
        function [l,p] = plotGeneral(obj, y, y_range)
            x = obj.X;
            l = plot(x,y);
            set(l,'DisplayName', 'Data');
            xlim([obj.Vars.lb obj.Vars.ub]);
            ylim(y_range);
            
            hold on
            % NaN Highlighting
            p1 = highlightGroups([isnan(y)], 'k');
            set(p1, 'DisplayName', 'Infeasible Areas');
            
            % Constraint Highlighting
            s = lambdaData(obj.OO);
            active = s.ineqnonlin > 0;
            p2 = highlightGroups(active(1,:),'b');
            set(p2, 'DisplayName', 'Propeller Constraint');
            
            p3 = highlightGroups(active(2,:),'r');
            set(p3, 'DisplayName', 'Motor Constraint');
            
            hold off
            
            p = [p1; p2; p3];
            
            function p = highlightGroups(index, color)
                index(end) = false;
                if any(index)
                    % Find Starts
                    A = [index false];
                    B = [false index];
                    start_i = A > B;
                    start_i = start_i(1:(end-1));
                    
                    % Find Ends
                    end_i = B > A;
                    end_i = [false end_i(2:(end-1))];
                    
                    starting_x = x(start_i);
                    ending_x = x(end_i);
                    
                    for i = 1:sum(start_i)
                        start_x = starting_x(i);
                        finish_x = ending_x(i);
                        p = patch([start_x start_x finish_x finish_x], [y_range fliplr(y_range)], color,'FaceAlpha',0.1,'EdgeColor','none');
                    end
                else
                    p = matlab.graphics.primitive.Patch.empty();
                end
            end
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
            
            l = {}; % Line Objects
            p = {}; % Patch Objects
            
            tile_counter = 1;
            CMat = zeros(N,N);
            for i = 1:N
                for j = 1:N
                    nexttile(tile_counter)
                    [CMat(j,i),~,y] = correlate(obj_array, j, i);
                    tile_counter = tile_counter+1;
                    
                    y_range = [lb(i) ub(i)];
                    [l{j,i}, p{j,i}] = plotGeneral(obj_array(j), y, y_range);
                    
                    if j == 1
                        ylabel(latex(vars(i), 'UnitFlag', false),lblspecs{:});
                    else
                        set(gca,'Yticklabel',[])
                    end
                    if i == numel(vars)
                        xlabel(latex(vars(j), 'UnitFlag', false), lblspecs{:});
                    else
                        set(gca,'Xticklabel',[])
                    end
                end
            end
            % Find unique line colors
            lines = vertcat(l{:});
            colors = vertcat(lines.Color);
            [~,unique_l_i] = unique(colors, 'rows');
            
            % Find unique patch colors
            patches = vertcat(p{:});
            colors = vertcat(patches.FaceColor);
            [~,unique_p_i] = unique(colors, 'rows');
            
            lgd = legend([lines(unique_l_i); patches(unique_p_i)]);
            lgd.Layout.Tile = 'east';
        end
        
        function [c,x,y] = correlate(obj_array, var1, var2)
            x = obj_array(var1).X;
            dd_array = [obj_array(var1).DD.(obj_array(var2).Vars.Parent.Name)];
            data = [dd_array.(obj_array(var2).Vars.Sym)];
            y = processCatData(obj_array(var1), data);
            m = [x',y'];
            m = m(all(~isnan(m),2),:);
            c = corrcoef(m);
            c = c(1,2);
        end
        
        function y = processCatData(obj,data)
            y = NaN(size(data,1),length(obj.X));
            y(:,obj.I) = data;
        end
        
    end
end

