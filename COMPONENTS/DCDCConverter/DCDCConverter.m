classdef DCDCConverter < Component
    %DCDCCONVERTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        L_1 {mustBeParam} = 1e-6;
        C_1 {mustBeParam} = 1e-6;
        L_3 {mustBeParam} = 1e-6;
        C_2 {mustBeParam} = 1e-6;
        R_1 {mustBeParam} = 1e-6;
    end
    
    methods (Access = protected)
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow("xt*xh");
            P(2) = Type_PowerFlow("u1*xt*xh");
            P(3) = Type_PowerFlow("xt^2");
            
            % Vertices

            % Internal Vertices
            desc = ["Inductance 1", "Capacitance 1", "Virtual Inductance", "Capacitance 2"];
            type = [VertexTypes.Current, VertexTypes.Voltage, VertexTypes.Current, VertexTypes.Voltage];
            cap = C(1);
            coeff = [obj.L_1, obj.C_1, obj.L_3, obj.C_2];
            init = 0;
            for i = 1:4
                V(i) = GraphVertex_Internal('Description',desc(i),'Capacitance',cap,'Coefficient',coeff(i),'Initial',init, 'VertexType', type(i));
            end
            
            % External Vertices
            desc = ["Output Current", "Input Voltage", "Heat Sink"];
            type = [VertexTypes.Current, VertexTypes.Voltage, VertexTypes.Temperature];
            init = [0 0 0];
            for i = 1:3
                V(i+4) = GraphVertex_External('Description',desc(i), 'VertexType', type(i));
            end
            
            % Inputs
            I(1) = GraphInput("d_1");
            I(2) = GraphInput("d_2");
            
            % Edges
            E(6) = GraphEdge();
            pflows = P([1,1,2,2,1,3]);
            coeffs = [1 1 1 1 1 obj.R_1];
            Emat = V([6 1;1 2;2 3;3 4;4 5;1 7]);
            for i = 1:6
                E(i) = GraphEdge_Internal('PowerFlow',pflows(i),'Coefficient',coeffs(i),'TailVertex',Emat(i,1),'HeadVertex',Emat(i,2));
            end
            E(3).Input = I(1);
            E(4).Input = I(2);
            
            g = Graph(V, E);
            obj.Graph = g;
            
            p(1) = ComponentPort('Description',"Voltage Input",'Element',obj.graph.Edges(1));
            p(2) = ComponentPort('Description',"Current Output",'Element',obj.graph.Edges(5));
            p(3) = ComponentPort('Description',"Heat Sink",'Element',obj.graph.Vertices(7));
            obj.Ports = p;
        end
    end
end

