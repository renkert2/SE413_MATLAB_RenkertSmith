classdef Battery < Component
    % Battery RC Specifications from: Nemes et. al. "Parameters identification using experimental measurements for equivalent circuit Lithium-Ion cell models"
        % Qulit Fire 18650 Li-Ion rechargeable cell with a rated capacity of 4500mAh, 3.7V rated, 4.2V maximum over charge voltage, 2.7V minimum discharge voltage.
    % Battery Pack specifications (Q and V_OCV) from: Ferry, "Quadcopter Plant Model and Control System Development With MATLAB/Simulink Implementation"
    
    properties
        Q double = 2520  %Battery Capacity - Coulombs
        C_1 double = 690.349 % Farads
        R_1 double = 0.003 % Ohms
        C_2 double = 1700.084 % Farads
        R_2 double = 0.0606 % Ohms
        R_s double = 0.09825 % Series Resistance - Ohms
        V_OCV double = 11.1 %Nominal Open Circuit Voltage, V_OCV(q)
    end
    
    methods (Access = protected)
        function g = DefineGraph(obj)
            % Capacitance Types
            C(1) = Type_Capacitance("1"); % Capacitance Type for Q*V_OCV
            C(2) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow("xh");
            P(2) = Type_PowerFlow("xt*xh");
            P(3) = Type_PowerFlow("xt^2");
            
            % Vertices
            Vertex(1) = GraphVertex_Internal('Description', "Battery SOC", 'Capacitance', C(1), 'Coefficient', obj.Q*obj.V_OCV, 'Initial', 1, 'VertexType','Abstract');
            Vertex(2) = GraphVertex_Internal('Description', "Capacitance 1", 'Capacitance', C(2), 'Coefficient', obj.C_1, 'Initial', 0, 'VertexType', 'Voltage');
            Vertex(3) = GraphVertex_Internal('Description', "Capacitance 2", 'Capacitance', C(2), 'Coefficient', obj.C_2, 'Initial', 0, 'VertexType', 'Voltage');
            Vertex(4) = GraphVertex_External('Description', "Load Current", 'Initial', 0, 'VertexType', 'Current');
            Vertex(5) = GraphVertex_External('Description', "Heat Sink", 'Initial', 0, 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            Edge(1) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',obj.V_OCV,'TailVertex',Vertex(1),'HeadVertex',Vertex(4));
            Edge(2) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',1/obj.R_1,'TailVertex',Vertex(2),'HeadVertex',Vertex(5));
            Edge(3) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',1/obj.R_2,'TailVertex',Vertex(3),'HeadVertex',Vertex(5));
            Edge(4) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(2));
            Edge(5) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(3));
            Edge(6) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',obj.R_s,'TailVertex',Vertex(4),'HeadVertex',Vertex(5));
            
            g = Graph(Vertex, Edge);
            obj.graph = g;

            % Ports
            p(1) = ComponentPort('Description','Load Current','Element', Vertex(4));
            obj.Ports = p;
        end
    end
end

