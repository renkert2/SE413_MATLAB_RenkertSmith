classdef ShaftCoupler < Component
    %SHAFTCOUPLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        k % Torsional spring constant
    end
    
    methods (Access = protected)
        function g = DefineGraph(obj)
%             g = GraphModel(2,3,2,0);
%             g.x_desc=["T"];
%             g.x_init = [0];
%             g.t_desc = ["\omega_1", "\omega_2"];
%             g.E = [2 1;1 3];
% 
%             g.V_type = [2 2 2];
%             g.V_val = [1/obj.k]; % Each V_val coefficient is multiplied by corresponding V_val_map
% 
%             g.P_f = @(x_t, x_h, u_j)..., % Power flow vector
%             [x_t*x_h;
%              x_t*x_h
%              ];
            
            % Capacitance Types
            
            % PowerFlow Types
            
            % Vertices
            
            % Inputs
            
            % Edges
            
            % Ports
            
            g = Graph(Vertex, Edge);
        end
    end
end

