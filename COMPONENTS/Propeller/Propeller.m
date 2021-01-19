classdef Propeller < Component
    %PROPELLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        J double = 5.7258e-04 % Rotational Inertia - kg*m^2
        k_Q double = 2.4968e-05 % Drag torque coefficient
        k_T double = 3.6494e-05 % Thrust coefficient
        rho double = 1.205 % Air Density - kg/m^3
        D double = 0.1270 % Propeller Diameter - m
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
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance('x');
            
            % PowerFlow Types
            P(1) = Type_PowerFlow('xt*xh');
            P(2) = Type_PowerFlow('xt^3');
            
            % Vertices
            V(1) = GraphVertex_Internal('Description', "Inertia (omega)",...
                'Capacitance', C(1),...
                'Coefficient', obj.J,...
                'Initial', 0,...
                'VertexType', 'AngularVelocity');
            
            V(2) = GraphVertex_External('Description', "Input Torque (T_m)", 'Initial',0,'VertexType','Torque');
            V(3) = GraphVertex_External('Description', "Drag Sink",'Initial',0,'VertexType','Abstract');
            
            % Inputs
            
            % Edges
            E(1) = GraphEdge_Internal(...
                'PowerFlow',P(1),...
                'Coefficient',1,...
                'TailVertex',V(2),...
                'HeadVertex',V(1));
            
            E(2) = GraphEdge_Internal(...
                'PowerFlow',P(2),...
                'Coefficient',obj.square_drag_coeff,...
                'TailVertex',V(1),...
                'HeadVertex',V(3));
                       
            g = Graph(V, E);
            obj.graph = g;
            
            % Ports
            p(1) = ComponentPort('Description',"Torque Input",'Element',E(1));
            obj.Ports = p;
        end
    end
end

