classdef Battery_Simple < Component
    % Battery Cell Specifications from: Nemes et. al. "Parameters identification using experimental measurements for equivalent circuit Lithium-Ion cell models"
    % Battery Pack specifications (N_s and N_p) from: Ferry, "Quadcopter Plant Model and Control System Development With MATLAB/Simulink Implementation"
    
    % All parameters specified per cell except for N_series and N_parallel
    
    properties
        Q {mustBeParam} = 7500  %Battery Capacity - Coulombs
        R_s {mustBeParam} = 0.09825 % Series Resistance - Ohms
        V_OCV {mustBeParam} = 3.7 %Nominal Open Circuit Voltage, V_OCV(q)
        
        N_s {mustBeParam} = 3 % Number of cells in series
        N_p {mustBeParam} = 1 % Number of cells in parallel
    end
    
    methods (Access = protected)
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance("1"); % Capacitance Type for Q*V_OCV
            C(2) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow("xh");
            P(2) = Type_PowerFlow("xt^2");
            
            % Vertices
            Vertex(1) = GraphVertex_Internal('Description', "Battery SOC", 'Capacitance', C(1), 'Coefficient', obj.N_s*obj.N_p*obj.Q*obj.V_OCV, 'Initial', 1, 'VertexType','Abstract');
            Vertex(2) = GraphVertex_External('Description', "Load Current", 'Initial', 0, 'VertexType', 'Current');
            Vertex(3) = GraphVertex_External('Description', "Heat Sink", 'Initial', 0, 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            Edge(1) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',obj.N_s*obj.V_OCV,'TailVertex',Vertex(1),'HeadVertex',Vertex(2));
            Edge(2) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',obj.N_s/obj.N_p*obj.R_s,'TailVertex',Vertex(2),'HeadVertex',Vertex(3));
            
            g = Graph(Vertex, Edge);
            obj.graph = g;

            % Ports
            p(1) = ComponentPort('Description','Load Current','Element', Vertex(2));
            obj.Ports = p;
        end
    end
end

