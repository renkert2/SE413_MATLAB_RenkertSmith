classdef Battery_Symbolic < Component
    % Battery Cell Specifications from: Nemes et. al. "Parameters identification using experimental measurements for equivalent circuit Lithium-Ion cell models"
    % Battery Pack specifications (N_s and N_p) from: Ferry, "Quadcopter Plant Model and Control System Development With MATLAB/Simulink Implementation"
    
    % All parameters specified per cell except for N_series and N_parallel
    
    properties
        Q = symParam('Q', 7500, ["positive","rational"])  %Battery Capacity - Coulombs
        C_1 = symParam('C_1', 690.349, ["positive","rational"]) % Farads
        R_1 = symParam('R_1', 0.003, ["positive","rational"]) % Ohms
        C_2 = symParam('C_2', 1700.084, ["positive","rational"]) % Farads
        R_2 = symParam('R_2', 0.0606, ["positive","rational"]) % Ohms
        R_s = symParam('R_s', 0.09825, ["positive","rational"]) % Series Resistance - Ohms
        V_OCV = symParam('V_OCV', 3.7, ["positive","rational"]) %Nominal Open Circuit Voltage, V_OCV(q)
        
        N_s = symParam('N_s', 3, ["positive","integer"]) % Number of cells in series
        N_p = symParam('N_p', 1, ["positive","integer"]) % Number of cells in parallel
    end
    
    methods (Access = protected)
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance("1"); % Capacitance Type for Q*V_OCV
            C(2) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow("xh");
            P(2) = Type_PowerFlow("xt*xh");
            P(3) = Type_PowerFlow("xt^2");
            
            % Vertices
            Vertex(1) = GraphVertex_Internal('Description', "Battery SOC", 'Capacitance', C(1), 'Coefficient', obj.N_s*obj.N_p*obj.Q*obj.V_OCV, 'Initial', 1, 'VertexType','Abstract');
            Vertex(2) = GraphVertex_Internal('Description', "Capacitance 1", 'Capacitance', C(2), 'Coefficient', obj.N_p/obj.N_s*obj.C_1, 'Initial', 0, 'VertexType', 'Voltage');
            Vertex(3) = GraphVertex_Internal('Description', "Capacitance 2", 'Capacitance', C(2), 'Coefficient', obj.N_p/obj.N_s*obj.C_2, 'Initial', 0, 'VertexType', 'Voltage');
            Vertex(4) = GraphVertex_External('Description', "Load Current", 'Initial', 0, 'VertexType', 'Current');
            Vertex(5) = GraphVertex_External('Description', "Heat Sink", 'Initial', 0, 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            Edge(1) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',obj.N_s*obj.V_OCV,'TailVertex',Vertex(1),'HeadVertex',Vertex(4));
            Edge(2) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',obj.N_p/obj.N_s*1/obj.R_1,'TailVertex',Vertex(2),'HeadVertex',Vertex(5));
            Edge(3) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',obj.N_p/obj.N_s*1/obj.R_2,'TailVertex',Vertex(3),'HeadVertex',Vertex(5));
            Edge(4) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(2));
            Edge(5) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',1,'TailVertex',Vertex(4),'HeadVertex',Vertex(3));
            Edge(6) = GraphEdge_Internal('PowerFlow',P(3),'Coefficient',obj.N_s/obj.N_p*obj.R_s,'TailVertex',Vertex(4),'HeadVertex',Vertex(5));
            
            g = Graph(Vertex, Edge);
            obj.graph = g;

            % Ports
            p(1) = ComponentPort('Description','Load Current','Element', Vertex(4));
            obj.Ports = p;
        end
    end
end

