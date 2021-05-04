classdef Optimization_ATC < handle  
    properties
        QR QuadRotor

        propMassFit propMassFit
        propAeroFit propAeroFit
        motorFit motorFit
        
        OptiVars (:,1) optiVar
        
        MinThrustRatio double = 2
        MaximumMass double = 1
    end
    
    methods
        function obj = Optimization_ATC(qr)
            if nargin == 0
                load QROpt.mat QROpt;
                qr = QROpt;
            end
            obj.QR = qr;
            obj.init();
        end
        
        function init(obj)            
            load PF_Aero.mat PF_Aero
            obj.propAeroFit = PF_Aero;
            
            load PF_Mass.mat PF_Mass
            obj.propMassFit = PF_Mass;
            
            load MF_KDE.mat MF_KDE
            obj.motorFit = MF_KDE;
                                    
            batt = obj.QR.Battery;
            prop = obj.QR.Propeller;
            motor = obj.QR.Motor;
            
            % Set Optimization Variables
            OV(1) = optiVar("D", obj.propAeroFit.Boundary.X_mean(1), obj.propAeroFit.Boundary.X_lb(1), obj.propAeroFit.Boundary.X_ub(1));
            OV(1).Description = "Diameter";
            OV(1).Unit = "m";
            OV(1).Parent = prop;
            
            OV(2) = optiVar("P", obj.propAeroFit.Boundary.X_mean(2), obj.propAeroFit.Boundary.X_lb(2), obj.propAeroFit.Boundary.X_ub(2));
            OV(2).Description = "Pitch";
            OV(2).Unit = "m";
            OV(2).Parent = prop;
            
            OV(3) = optiVar("N_p", 1, 0.1, 10);
            OV(3).Description = "Parallel Cells";
            OV(3).Parent = batt;
            
            OV(4) = optiVar("Kt", 0.01, 1e-6, 10); % Check These
            OV(4).Description = "Torque Constant";
            OV(4).Unit = "N*m/A";
            OV(4).Parent = motor;
            
            OV(5) = optiVar("Rm",0.01,1e-6,10); % Check These
            OV(5).Description = "Phase Resistance";
            OV(5).Unit = "Ohm";
            OV(5).Parent = motor;
            
            OV(6) = optiVar("Mm", 0.01, 1e-6, 10);
            OV(6).Description = "Motor Mass";
            OV(6).Unit = "Kg";
            OV(6).Parent = motor;

            obj.OptiVars = OV';
        end
        
        function Optimize_ATC(obj)
            % Options
            beta = 1.3; epsilon = 1e-4; max_iter = 100;
            
            % Intialize Penalty Weights:
            % v = [v_q v_m]
            % w = [w_q w_m]
            
            v = zeros(3,2);
            w = ones(3,2);
            
            % Initialize Y_bar_m
            Y_bar_m = struct();
            Y_bar_m.K_tau = 0.005;
            Y_bar_m.R_phase = 0.13;
            Y_bar_m.active_mass = 0.125;
           
            c_old = [inf inf];
            for i = 1:max_iter
                % Run inner ATC
                v_q = v(:,1);
                w_q = w(:,1);
                [Y_bar_q, C_q, Z] = ATC_upper_function(obj, Y_bar_m, v_q, w_q);
                
                v_m = v(:,2);
                w_m = w(:,2);
                [Y_bar_m, C_m] = ATC_lower_function(Z, Y_bar_q, v_m, w_m);
                
                % Create Consistency Constraint Matrix c
                c_q = [C_q.K_tau; C_q.R_phase; C_q.active_mass];
                c_m = [C_m.K_tau; C_m.R_phase; C_m.active_mass];
                c = [c_q c_m];
                
                % Evaluate termination criteria
                if norm(c-c_old) <= epsilon
                    if norm(c) <= epsilon
                        break;
                    end
                end
                c_old = c;
                
                %Update Penalty Weights
                v = v + 2*w.*w.*c;
                w = w*beta;
            end
        end

        function [Y_bar_q, C_q, Z] = ATC_upper_function(obj, Y_bar_m, v_q, w_q, opts)
            arguments
                obj
                Y_bar_m % Coupling Variables - Motor Copy
                v_q % Linear coupling variable weights 
                w_q % Quadratic coupling variable weights
                opts.FlightTimeOpts cell = {}
                opts.OptimizationOpts cell = {}
                opts.OptimizationOutput logical = true
                opts.InitializeFromValue logical = true
            end
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp');
            if opts.OptimizationOutput
                optimopts = optimoptions(optimopts, 'Display', 'iter-detailed', 'PlotFcn', {@optimplotx,@optimplotfval,@optimplotfirstorderopt});
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});
            
            if opts.InitializeFromValue
                x0 = scale(obj.OptiVars);
            else
                x0 = X0(obj.OptiVars);
            end
            
            lb = LB(obj.OptiVars);
            ub = UB(obj.OptiVars);
            
            % Turn Y_bar_m into vector
            y_bar_m(1,1) = Y_bar_m.K_tau;
            y_bar_m(2,1) = Y_bar_m.R_phase;
            y_bar_m(3,1) = Y_bar_m.active_mass;

            [X_opt_s, fval, exitflag, ~, lambda] = fmincon(@objfun ,x0, [], [], [], [], lb, ub, @nlcon, optimopts);
            
            % Get additional ouptuts at optimal point and convert back to struct
            [~, y_bar_q, c_q] = objfun(X_opt_s);
            Y_bar_q = struct();
            Y_bar_q.K_tau = y_bar_q(1);
            Y_bar_q.R_phase = y_bar_q(2);
            Y_bar_q.active_mass = y_bar_q(3);
            
            C_q = struct();
            C_q.K_tau = c_q(1);
            C_q.R_phase = c_q(2);
            C_q.active_mass = c_q(3);
            
            Z = [];
            
            % Set Current Values to Optimal Value in OptiVars
            setVals(obj.OptiVars, X_opt_s);
            
            % Ensure the QuadRotor gets updated to the correct sym param vals
            obj.updateParamVals(XAll(obj.OptiVars));    
            
            function [f, y_bar_q, c_q] = objfun(X_s)
                X = XAll(obj.OptiVars,X_s); % Unscale and return all
                try
                    obj.updateParamVals(X);
                    ft = -obj.QR.flightTime();
                    
                    y_bar_q = X(find(obj.OptiVars, ["Kt", "Rm", "Mm"]));
                    c_q = y_bar_q - y_bar_m;
                    phi_q = dot(v_q,c_q) + norm(w_q.*c_q)^2;
                     
                    f = ft/5000 + phi_q;
                catch
                    f = NaN;
                end
            end
            
            function [c,ceq] = nlcon(X_s)
                X = XAll(obj.OptiVars,X_s); % Unscale and return all values                
                
                try
                    obj.updateParamVals(X);
                    % Constraint 1: D,P In Valid Region
                    c_1 = distToBoundary(obj.propAeroFit.Boundary, X(find(obj.OptiVars, ["D", "P"])));
                    % Constraint 2: Thrust Ratio Constraint
                    c_2 = (obj.MinThrustRatio - obj.QR.calcThrustRatio());
                    % Constraint 3: Mass Constraint
                    c_3 = obj.QR.Mass() - obj.MaximumMass; 
                    
                    c = [c_1;c_2;c_3];
                catch
                    c = NaN(3,1);
                end
                
                ceq = [];
            end
        end
        
        function resetParamVals(obj)
           obj.updateParamVals(vertcat(obj.OptiVars.x0)); 
        end
        
        function updateParamVals(obj,X)
            %X_prop = [D;P]
            X_prop = X(find(obj.OptiVars, ["D", "P"]));
            D_prop = X_prop(1);
            P_prop = X_prop(2);
            [k_P_prop, k_T_prop] = calcPropCoeffs(obj.propAeroFit, X_prop);
            [M_prop,J_prop] = calcMassProps(obj.propMassFit, D_prop);
                   
            %X_batt = [N_p; N_s]
            X_batt = X(find(obj.OptiVars, ["N_p"]));
            N_p_batt = X_batt(1);
            
            %X_motor = [Kt; Rm, Mm]
            X_motor = X(find(obj.OptiVars, ["Kt", "Rm", "Mm"]));
            K_t_motor = X_motor(1);
            Rm_motor = X_motor(2);
            M_motor = X_motor(3);
            
            
            QR = obj.QR;
            
            % Battery
            QR.Battery.N_p.Value = N_p_batt;
            
            % Prop
            QR.Propeller.D.Value = D_prop;
            QR.Propeller.P.Value = P_prop;
            QR.Propeller.J.Value = J_prop;
            QR.Propeller.M.Value = M_prop;
            QR.Propeller.k_P.Value = k_P_prop;
            QR.Propeller.k_T.Value = k_T_prop;
            
            % Motor
            QR.Motor.K_t.Value = K_t_motor;
            QR.Motor.Rm.Value = Rm_motor;
            QR.Motor.M.Value = M_motor;
             
            obj.QR.update();
        end
    end
end

