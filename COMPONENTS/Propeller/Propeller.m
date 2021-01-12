classdef Propeller < Component
    %PROPELLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        J % Rotational Inertia
        k_Q % Drag torque coefficient
        k_T % Thrust coefficient
        rho % Air Density
        D % Propeller Diameter
    end
    
    properties (Dependent)
        square_drag_coeff %coefficient in front of speed^2 term, N*m/(rev/s)^2.
        square_thrust_coeff %coefficient in front of speed^2 term, N/(rev/s)^2.
    end
    
    methods
        function sdc = get.square_drag_coeff(obj)
            sdc = obj.k_Q*obj.rho*obj.D^5;
        end
        function stc = get.square_thrust_coeff(obj)
            stc = obj.k_T*obj.rho*obj.D^4;
        end
    end
    
    methods (Access = protected)
        function g=DefineGraph(obj)
            %             g = GraphModel(2,3,2,0);
            %
            %             g.x_desc=["\omega"];
            %             g.t_desc=["T_m", "T_d"];
            %             g.x_init = [0];
            %             g.E = [2 1;1 3];
            %
            %             g.V_type = [2 2 2];
            %             g.V_val = [p.J]; % Each V_val coefficient is multiplied by corresponding V_val_map
            %
            %             g.P_f = @(x_t, x_h, u_j)..., % Power flow vector
            %             [x_t*x_h;
            %              p.square_drag_coeff*x_t^3;
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

