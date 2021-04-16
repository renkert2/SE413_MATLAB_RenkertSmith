classdef UI_LabeledValue < matlab.ui.componentcontainer.ComponentContainer
    %UI_LABELEDVALUE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Label string = "Label"
        Value double = 0.0
        Unit string = ""
    end
    
    properties (Access = private, Transient, NonCopyable) 
       Grid matlab.ui.container.GridLayout
       Label_ matlab.ui.control.Label
       Value_ matlab.ui.control.Label
       Unit_ matlab.ui.control.Label
    end
    
    methods (Access = protected)
        function setup(obj)
            obj.Grid = uigridlayout(obj, [1 3]);
            obj.Grid.ColumnWidth = {'fit','fit','fit'};
            obj.Grid.RowHeight = ["fit"];
            
            obj.Label_ = uilabel(obj.Grid);
            obj.Label_.HorizontalAlignment = 'right';
            obj.Label_.FontSize = 14;
            obj.Label_.FontWeight = 'bold';
            
            
            obj.Value_ = uilabel(obj.Grid);
            obj.Value_.Interpreter = 'latex';
            obj.Value_.HorizontalAlignment = 'left';
            
            
            obj.Unit_ = uilabel(obj.Grid);
            obj.Unit_.Interpreter = 'latex';
            obj.Unit_.HorizontalAlignment = 'left';
        end
        
        function update(obj)
            obj.Label_.Text = obj.Label+": ";
            obj.Value_.Text = compose("%.2f", obj.Value);
            obj.Unit_.Text = obj.Unit;
        end

    end
end

