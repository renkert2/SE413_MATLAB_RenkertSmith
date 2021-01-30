classdef PMSMInverter < Component
    %DCDCCONVERTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        InverterType InverterTypes = InverterTypes.LinearLoss
        
        rated_current double = 20; %Rated Current - Amps
        nominal_voltage double = 14.8; %Nominal (average) voltage - Volts
        eta = 0.85; % Efficiency
        
        L double = 0;
        C double = 0;
        R_1 double = 0.0;
        R_2 double = 1.5;
    end
    
    methods
        function r = setResistance(obj, rated_current, nominal_voltage, eta)
            % Placeholder - would like to calculate R_1 and R_2 given rated values and efficiency here
        end
    end
    
    methods (Access = protected)
        function init(obj)
            obj.setResistance();
        end
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance("x");
            
            % PowerFlow Types
            P(1) = Type_PowerFlow("xt*xh");
            P(2) = Type_PowerFlow("u1*xt*xh");
            P(3) = Type_PowerFlow("xt^2");
            
            % Vertices

            % Internal Vertices
            desc = ["DC Inductance", "q Capacitance"];
            type = [VertexTypes.Current, VertexTypes.Voltage];
            cap = C(1);
            coeff = [obj.L, obj.C];
            init = 0;
            for i = 1:2
                V(i) = GraphVertex_Internal('Description',desc(i),'Capacitance',cap,'Coefficient',coeff(i),'Initial',init, 'VertexType', type(i));
            end
            
            % External Vertices
            desc = ["Input Voltage (DC)", "Output Current (q)", "Heat Sink"];
            type = [VertexTypes.Voltage, VertexTypes.Current, VertexTypes.Temperature];
            init = [0 0 0];
            for i = 1:3
                V(i+2) = GraphVertex_External('Description',desc(i),'Initial', init(i), 'VertexType', type(i));
            end
            
            % Inputs
            I(1) = GraphInput("d");
            
            % Edges
            E(1) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',sqrt(3/2),'TailVertex',V(3),'HeadVertex',V(1), 'Input', I(1));
            E(2) = GraphEdge_Internal('PowerFlow',P(1),'Coefficient',1,'TailVertex',V(1),'HeadVertex',V(2));
            E(3) = GraphEdge_Internal('PowerFlow',P(2),'Coefficient',sqrt(3/2),'TailVertex',V(2),'HeadVertex',V(4), 'Input', I(1));
            
            if obj.InverterType == InverterTypes.ConstantLoss
                P(4) = Type_PowerFlow("xt");
                E(4) = GraphEdge_Internal('PowerFlow',[P(3) P(4)],'Coefficient',[obj.R_1 obj.R_2],'TailVertex',V(1),'HeadVertex',V(5));
            elseif obj.InverterType == InverterTypes.LinearLoss
                P(4) = Type_PowerFlow("u1*xt");
                E(4) = GraphEdge_Internal('PowerFlow',[P(3) P(4)],'Coefficient',[obj.R_1 obj.R_2],'TailVertex',V(1),'HeadVertex',V(5), 'Input', I(1));
            else
                error('Invalid InverterType')
            end
            
            
            g = Graph(V, E);
            obj.graph = g;
            
            p(1) = ComponentPort('Description',"Voltage Input (DC)",'Element',obj.graph.Edges(1));
            p(2) = ComponentPort('Description',"Current Output (q)",'Element',obj.graph.Edges(3));
            p(3) = ComponentPort('Description',"Heat Sink",'Element',obj.graph.Vertices(5));
            obj.Ports = p;
        end
    end
end

