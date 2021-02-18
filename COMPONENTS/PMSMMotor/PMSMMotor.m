classdef PMSMMotor < Component
    %PMSM Motor
    %   Default parameters from Ferry 2017 - 'Quadcopter Plant Model and Control System Development With MATLAB/Simulink Implementation'
    
    properties
        L {mustBeParam} = 1.17e-4 % Leakage Inductance - H
        J {mustBeParam} = 6.5e-6 % Mechanical rotational inertia - Modified to better reflect Ferry's simulation results
        K_t {mustBeParam} = 0.00255 % Torque/Speed Coefficient - Nm/A
        R_1 {mustBeParam} = 0.117 % Phase Resistance - Ohms
        B_v {mustBeParam} = 2.415e-6 % Viscous Friction - N*m*s
        T_c {mustBeParam} = 0 % Coulomb Friction
        sigmoid_a_param {mustBeParam} = 10 % Parameter used to approximate sign function with sigmoid function sig(w) = 2/(1+Exp(-a*w))-1
    end
    
    methods (Static)
        function K_t = calcTorqueConstant(P,lambda_m)
            % P - Total number of poles - not pole pairs
            % lambda_m - Magnetic Flux Linkage
            K_t = (P/2)*lambda_m;
        end
    end
    
    
    methods (Access = protected)
        function DefineComponent(obj)
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
            
            V(3) = GraphVertex_External('Description', "Input Voltage (v_q)", 'VertexType', 'Voltage');
            V(4) = GraphVertex_External('Description', "Mechanical Load (T_l)", 'VertexType', 'Torque');
            V(5) = GraphVertex_External('Description', "Heat Sink", 'VertexType', 'Temperature');
            
            % Inputs
            
            % Edges
            E(1) = GraphEdge_Internal(...
                'PowerFlow',PF(1),...
                'Coefficient',1,...
                'TailVertex',V(3),...
                'HeadVertex',V(1));
            
            E(2) = GraphEdge_Internal(...
                'PowerFlow',PF(1),...
                'Coefficient',sqrt(3/2)*obj.K_t,...
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
            obj.Graph = g;
            
            % Ports            
            p(1) = ComponentPort('Description',"Voltage Input",'Element',E(1));
            p(2) = ComponentPort('Description',"Torque Output",'Element',E(3));
            p(3) = ComponentPort('Description',"Heat Sink",'Element',V(5));
            obj.Ports = p;
        end
    end
end

