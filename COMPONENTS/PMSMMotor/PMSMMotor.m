classdef PMSMMotor < Component
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        L % Leakage Inductance
        J % Mechanical rotational inertia
        P % Total number of poles
        lambda_m % Magnetic Flux Linkage
        R_1 % Phase Resistance
        B_v % Viscous Friction
        T_c % Coulomb Friction
        sig = 1 % Sigmoid function of speed sig(w), LUT at some point
    end
    
    methods (Access = protected)
        function g=DefineGraph(p)
            %             g = GraphModel(6,6,4,0);
            %
            %             g.x_desc=["i_q" "\omega_m"];
            %             g.t_desc = ["v_q", "T_l", "T_1", "T_2"];
            %             g.x_init = [0 0];
            %             g.E = [3 1;1 2;2 4;1 5;2 6;2 6];
            %
            %             g.V_type = [2 2 2 2 1 1];
            %             g.V_val = [p.L, p.J]; % Each V_val coefficient is multiplied by corresponding V_val_map
            %
            %             g.P_f = @(x_t, x_h, u_j)... % Power flow vector
            %                 [x_t*x_h;
            %                 x_t*x_h*p.P/2*sqrt(3/2)*p.lambda_m;
            %                 x_t*x_h;
            %                 x_t^2*p.R_1;
            %                 p.B_v*x_t^2;
            %                 p.T_c*x_t;
            %                 ];
            %
            %             g.set_g(@(x_t, x_h, u) p.sig(x_t),6); % Add sigmoid function to the last power flow coefficient
                        
            % Capacitance Types
            
            % PowerFlow Types
            
            % Vertices
            
            % Inputs
            
            % Edges
            
            % Ports
            
            g = Graph(Vertex, Edge);
        end
    end
end

