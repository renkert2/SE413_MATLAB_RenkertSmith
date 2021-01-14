classdef PMSMMotor < Component
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        L double {mustBeNonnegative} = 1 % Leakage Inductance
        J double {mustBeNonnegative} = 1% Mechanical rotational inertia
        P double {mustBeInteger, mustBeNonnegative} = 12 % Total number of poles
        lambda_m double {mustBeNonnegative} = 1% Magnetic Flux Linkage
        R_1 double {mustBeNonnegative} = 1 % Phase Resistance
        B_v double {mustBeNonnegative} = 1 % Viscous Friction
        T_c double {mustBeNonnegative} = 1 % Coulomb Friction
        sigmoid_a_param double = 10 % Parameter used to approximate sign function with sigmoid function sig(w) = 2/(1+Exp(-a*w))-1
    end
    
    methods (Access = protected)
        function g=DefineGraph(obj)
            % Capacitance Types
            C(1) = Type_Capacitance('x');
            
            % PowerFlow Types
            PF(1) = Type_PowerFlow('xt*xh');
            PF(2) = Type_PowerFlow('xt^2');
            
            syms xt
            sig = 2/(1+exp(-obj.sigmoid_a_param.*xt))-1;
            PF(3) = Type_PowerFlow(sig*xt);
            
            % Vertices
            V(1) = GraphVertex_Internal('Description', "Inductance (i_q)", 'Capacitance', C(1), 'Coefficient', obj.L, 'Initial', 0, 'VertexType', 'Current');
            V(2) = GraphVertex_Internal('Description', "Inertia (omega_m)", 'Capacitance', C(1), 'Coefficient', obj.J, 'Initial', 0, 'VertexType', 'AngularVelocity');
            
            V(3) = GraphVertex_External('Description', "Input Voltage (v_q)",'Initial',0, 'VertexType', 'Voltage');
            V(4) = GraphVertex_External('Description', "Mechanical Load (T_l)",'Initial',0, 'VertexType', 'Torque');
            V(5) = GraphVertex_External('Description', "Heat Sink",'Initial',0, 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            E(1) = GraphEdge_Internal(...
                'PowerFlow',PF(1),...
                'Coefficient',1,...
                'TailVertex',V(3),...
                'HeadVertex',V(1));
            
            E(2) = GraphEdge_Internal(...
                'PowerFlow',PF(1),...
                'Coefficient',obj.P/2*sqrt(3/2)*obj.lambda_m,...
                'TailVertex',V(1),...
                'HeadVertex',V(2));
            
            E(3) = GraphEdge_Internal(...
                'PowerFlow',PF(1),...
                'Coefficient',1,...
                'TailVertex',V(2),...
                'HeadVertex',V(4));
            
            E(4) = GraphEdge_Internal(...
                'PowerFlow',PF(2),...
                'Coefficient',obj.R_1,...
                'TailVertex',V(1),...
                'HeadVertex',V(5));
            
            E(5) = GraphEdge_Internal(...
                'PowerFlow',[PF(2) PF(3)],...
                'Coefficient',[obj.B_v obj.T_c],...
                'TailVertex',V(2),...
                'HeadVertex',V(5));
                       
            g = Graph(V, E);
            obj.graph = g;
            
            % Ports            
            p(1) = ComponentPort('Description',"Voltage Input",'Element',E(1));
            p(2) = ComponentPort('Description',"Torque Output",'Element',E(3));
            p(3) = ComponentPort('Description',"Heat Sink",'Element',V(5));
            obj.Ports = p;
        end
    end
end

