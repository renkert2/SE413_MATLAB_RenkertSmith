classdef ShaftCoupler < Component
    %SHAFTCOUPLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        k {mustBeParam} = 200e3 % Torsional spring constant - N*m/rad
    end
    
    methods (Access = protected)
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance('x');
            
            % PowerFlow Types
            P(1) = Type_PowerFlow('xt*xh');
            
            % Vertices
            V(1) = GraphVertex_Internal('Description', "Torque (T)", 'Capacitance', C(1), 'Coefficient', (1/obj.k), 'Initial', 0, 'VertexType', 'Torque');
            
            V(2) = GraphVertex_External('Description', "Input Inertia (omega_1)", 'VertexType', 'AngularVelocity');
            V(3) = GraphVertex_External('Description', "Output Inertia (omega_2)", 'VertexType', 'AngularVelocity');
            
            % Inputs
            
            % Edges
            E(1) = GraphEdge_Internal(...
                'PowerFlow',P(1),...
                'Coefficient',1,...
                'TailVertex',V(2),...
                'HeadVertex',V(1));
            
            E(2) = GraphEdge_Internal(...
                'PowerFlow',P(1),...
                'Coefficient',1,...
                'TailVertex',V(1),...
                'HeadVertex',V(3));
            
            g = Graph(V,E);
            obj.Graph = g;
            
            % Ports
            p(1) = ComponentPort('Description',"Inertia Input",'Element',E(1));
            p(2) = ComponentPort('Description',"Inertial Output",'Element',E(2));
            obj.Ports = p;
        end
    end
end

