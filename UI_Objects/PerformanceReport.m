classdef PerformanceReport < matlab.ui.componentcontainer.ComponentContainer
    %DESIGNREPORT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        PD PerformanceData
    end
    
    properties (Access = private)
        eta UI_LabeledValue
        ft UI_LabeledGauge
    end
      
    methods
        function obj = PerformanceReport(PD,parent,varargin)
            arguments
                PD PerformanceData
                parent = []
            end
            arguments (Repeating)
                varargin
            end
            
            if isempty(parent)
                fig = uifigure('Name', 'QuadRotor Design Report');
                parent = uigridlayout(fig, [1 1]);
                parent.RowHeight = ["1x"];
                parent.ColumnWidth = ["1x"];
            end
            
            obj = obj@matlab.ui.componentcontainer.ComponentContainer(parent,'PD', PD, varargin{:});
        end
        
        function updatePublic(obj)
            % Flight Time
            ft = obj.PD.FlightTime;
            obj.ft.Value = ft;
            
            % Efficiency
            eta_struct = PD.Efficiency;
            obj.eta(1).Value = eta_struct.Battery;
            obj.eta(2).Value = eta_struct.Inverter;
            obj.eta(3).Value = eta_struct.Motor;
            obj.eta(4).Value = eta_struct.Sys;
        end
    end
    
    methods (Access = protected)
        function setup(obj)
            grid = uigridlayout(obj, [4 2]);
            grid.RowHeight = ["fit", "fit", "fit", "fit"];
            grid.ColumnWidth = ["1x", "1x"];
            
            % Flight Time
            ft_panel = uipanel(grid, 'Title', 'Flight Time');
            ft_panel.Layout.Column = 1;
            ft_panel.Layout.Row = 1;
            obj.ft = UI_LabeledGauge(ft_panel, 'Label', 'Flight Time', 'Unit', "min", 'Limits', [0 100]);
            
            % System Quantities
            quant_panel = uipanel(grid, 'Title', 'System Quantities');
            quant_panel.Layout.Column = 1;
            quant_panel.Layout.Row = 2;
            
            % Efficiency
            eta_panel = uipanel(grid, 'Title', 'System Efficiency');
            eta_panel.Layout.Column = 1;
            eta_panel.Layout.Row = 3;
            
            eta_grid = uigridlayout(eta_panel, [4 1]);
            obj.eta(1) = UI_LabeledValue(eta_grid, 'Label', "Battery", 'Unit', "\%");
            obj.eta(2) = UI_LabeledValue(eta_grid, 'Label', "Inverter", 'Unit', "\%");
            obj.eta(3) = UI_LabeledValue(eta_grid, 'Label', "Motor", 'Unit', "\%");
            obj.eta(4) = UI_LabeledValue(eta_grid, 'Label', "System", 'Unit', "\%");
            
            % System Steady-State
            ss_panel = uipanel(grid, 'Title', 'System Steady State');
            ss_panel.Layout.Column = 1;
            ss_panel.Layout.Row = 4;
        end
        
        function update(obj)
            updatePublic(obj);
        end
    end
end

