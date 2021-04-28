classdef UI_LabeledGauge < matlab.ui.componentcontainer.ComponentContainer
    %UI_LABELEDVALUE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Label string = "Label"
        Value double = 0.0
        Limits double = [0 100]
        Unit string = ""
    end
    
    properties (Access = private, Transient, NonCopyable) 
       Grid matlab.ui.container.GridLayout
       Gauge matlab.ui.control.Gauge
       Label_ UI_LabeledValue
    end
    
    methods (Access = protected)
        function setup(obj)
            obj.Grid = uigridlayout(obj, [2 1]);
            obj.Grid.ColumnWidth = ["1x"];
            obj.Grid.RowHeight = ["1x", "1x"];
            
            obj.Gauge = uigauge(obj.Grid);
            obj.Gauge.Layout.Row = 1;
            
            obj.Label_ = UI_LabeledValue(obj.Grid, 'Label', obj.Label, 'Unit', obj.Unit);
            obj.Label_.Layout.Row = 2;
        end
        
        function update(obj)
            obj.Gauge.Value = obj.Value;
            obj.Gauge.Limits = obj.Limits;
            obj.Label_.Value = obj.Value;
        end

    end
end

