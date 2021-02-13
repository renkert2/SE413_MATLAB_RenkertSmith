classdef Propeller < Component
    %PROPELLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        J {mustBeParam} = 4.4e-05 % Rotational Inertia - kg*m^2 from "Stabilization and Control of Unmanned Quadcopter (Jiinec)
        k_Q {mustBeParam} = 0.03/(2*pi) % Drag torque coefficient - N/(s*kg*m)=1/s^4, speed in rev/s.
        k_T {mustBeParam} = 0.05 % Thrust coefficient - N/(s^2*kg*m^2), speed in rev/s.
        rho {mustBeParam} = 1.205 % Air Density - kg/m^3
        D {mustBeParam} = 0.1270 % Propeller Diameter - m
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
        function k_Q = setTorqueCoeff(obj, lumped_torque_coeff)
            k_Q = lumped_torque_coeff/(obj.rho*obj.D.^5);
            obj.k_Q = k_Q;
        end
        function k_T = setThrustCoeff(obj, lumped_thrust_coeff)
            k_T = lumped_thrust_coeff/(obj.rho*obj.D.^4);
            obj.k_T = k_T;
        end
    end
    
    methods (Access = protected)
        function init(obj)

        end
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance('x');
            
            % PowerFlow Types
            P(1) = Type_PowerFlow('xt*xh');
            P(2) = Type_PowerFlow('(xt/(2*pi))^3');
            
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

