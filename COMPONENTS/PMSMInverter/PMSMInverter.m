classdef PMSMInverter < DCDCConverter
    %DCDCCONVERTER Summary of this class goes here
    %   Detailed explanation goes here
    methods
        function obj = PMSMInverter(varargin)
             obj = obj@DCDCConverter(varargin{:}); 
             desc = ["DC Inductance", "DC Capacitance", "Virtual Inductance", "q Capacitance"];
             state_verts = obj.graph.Vertices(1:4);
             
             for i=1:length(state_verts)
                 state_verts(i).Description = desc(i);
             end
             
             obj.graph.Edges(3).Coefficient = sqrt(3/2);
             
             obj.graph.init();
        end
    end
end

