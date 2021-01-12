classdef Battery < Component
    
    properties
        Q double = 1  %Battery Capacity
        C_1 double = 1
        R_1 double = 1
        C_2 double = 1
        R_2 double = 1
        L_1 double = 1
        R_s double = 1 %Series Reistance
        V_OCV double = 1 %Open Circuit Voltage, V_OCV(q).  Will become a LUT at some point
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
            Vertex(1) = GraphVertex_Internal('Description', "Battery SOC", 'Capacitance', C(1), 'Coefficient', obj.Q*obj.V_OCV, 'Initial', 1);
            Vertex(2) = GraphVertex_Internal('Description', "Capacitance 1", 'Capacitance', C(2), 'Coefficient', obj.C_1, 'Initial', 0);
            Vertex(3) = GraphVertex_Internal('Description', "Capacitance 2", 'Capacitance', C(2), 'Coefficient', obj.C_2, 'Initial', 0);
            Vertex(4) = GraphVertex_Internal('Description', "Inductance 1", 'Capacitance', C(2), 'Coefficient', obj.L_1, 'Initial', 0);
            Vertex(5) = GraphVertex_External('Description', "Load Voltage", 'Initial', 0);
            Vertex(6) = GraphVertex_External('Description', "Heat Sink", 'Initial', 0);
            
            % Inputs
            
            % Edges
            Edge(1) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',obj.V_OCV,'TailVertex',Vertex(1),'HeadVertex',Vertex(4));
            Edge(2) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',1/obj.R_1,'TailVertex',Vertex(2),'HeadVertex',Vertex(6));
            Edge(3) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',1/obj.R_2,'TailVertex',Vertex(3),'HeadVertex',Vertex(6));
            Edge(4) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(2));
            Edge(5) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(3));
            Edge(6) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(5));
            Edge(7) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',obj.R_s,'TailVertex',Vertex(4),'HeadVertex',Vertex(6));
            
            g = Graph(Vertex, Edge);
            obj.graph = g;

            % Ports
            p(1) = ComponentPort('Description','Load Voltage','Domain','Electrical','Element', Edge(6));
            obj.Ports = p;
        end
    end
end

