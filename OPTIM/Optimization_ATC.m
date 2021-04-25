classdef Optimization_ATC < handle  
    properties
        QR QuadRotor

        propMassFit propMassFit
        propAeroFit propAeroFit
        motorFit motorFit
        
        OptiVars (:,1) optiVar
        
        MinThrustRatio double = 2
    end
    
    methods
        function obj = Optimization_ATC()
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
            OV(1) = optiVar("D", obj.propAeroFit.Boundary.X_mean(1), obj.propAeroFit.Boundary.X_lb(1), 0.35);
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
            
            OV(4) = optiVar("N_s", 3, 1, 12);
            OV(4).Description = "Series Cells";
            OV(4).Parent = batt;
            OV(4).Value = 6;
            OV(4).Enabled = false; % Higher voltages are always best
            
            OV(5) = optiVar("Kt", 0.01, 1e-6, 10); % Check These
            OV(5).Description = "Torque Constant";
            OV(5).Unit = "N*m/A";
            OV(5).Parent = motor;
            
            OV(6) = optiVar("Rm",0.01,1e-6,10); % Check These
            OV(6).Description = "Phase Resistance";
            OV(6).Unit = "Ohm";
            OV(6).Parent = motor;
            
            OV(7) = optiVar("Mm", 0.01, 1e-6, 10);
            OV(7).Description = "Motor Mass";
            OV(7).Unit = "Kg";
            OV(7).Parent = motor;

            obj.OptiVars = OV';
        end

        function [X_opt_s, opt_flight_time, output] = Optimize(obj,opts)
            arguments
                obj
                opts.FlightTimeOpts cell = {}
                opts.OptimizationOpts cell = {}
                opts.OptimizationOutput logical = true
                opts.InitializeFromValue logical = false
            end
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp');
            if opts.OptimizationOutput
                optimopts = optimoptions(optimopts, 'Display', 'iter-detailed', 'PlotFcn', {@optimplotx,@optimplotfval,@optimplotfirstorderopt});
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});
            
            if opts.InitializeFromValue
                x0 = .75*scale(obj.OptiVars) + .25*X0(obj.OptiVars);
            else
                x0 = X0(obj.OptiVars);
            end
            lb = LB(obj.OptiVars);
            ub = UB(obj.OptiVars);

            [X_opt_s, fval, ~, output] = fmincon(@objfun ,x0, [], [], [], [], lb, ub, @nlcon, optimopts);
            opt_flight_time = -fval;
            
            % Set Current Values to Optimal Value in OptiVars
            setVals(obj.OptiVars, X_opt_s);
            
            % Ensure the QuadRotor gets updated to the correct sym param vals
            obj.updateParamVals(XAll(obj.OptiVars));
            
            % Update QuadRotor's Flight Time to the optimal value
            obj.QR.flight_time = opt_flight_time;     
            
            function f = objfun(X_s)
                %objfun(X_s, Y_bar_m, v_q, w_q)
                X = XAll(obj.OptiVars,X_s); % Unscale and return all
                try
                    obj.updateParamVals(X);
                    ft = -obj.QR.flightTime('SimulationBased',false, opts.FlightTimeOpts{:});
                    
%                     Y_bar_q = X(find(obj.OptiVars, ["Kt", "Rm", "Mm"]));
%                     c_q = Y_bar_q - Y_bar_m;
%                     phi_q = v_q*c_q + (w_q*c_q)^2;
%                     
%                     f = ft + phi_q;
                    f = ft;
                catch
                    f = NaN;
                end
            end
            
            function [c,ceq] = nlcon(X_s)
                X = XAll(obj.OptiVars,X_s); % Unscale and return all values
                % Boundary Objectives
                c_1 = distToBoundary(obj.propAeroFit.Boundary, X(find(obj.OptiVars, ["D", "P"])));
                
                try
                    obj.updateParamVals(X);
                    c_2 = (obj.MinThrustRatio - obj.QR.calcThrustRatio());
                catch
                    c_2 = NaN;
                end
                
                c = [c_1;c_2];
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
            X_batt = X(find(obj.OptiVars, ["N_p", "N_s"]));
            N_p_batt = X_batt(1);
            N_s_batt = X_batt(2);
            
            %X_motor = [Kt; Rm, Mm]
            X_motor = X(find(obj.OptiVars, ["Kt", "Rm", "Mm"]));
            K_t_motor = X_motor(1);
            Rm_motor = X_motor(2);
            M_motor = X_motor(3);
            
            
            QR = obj.QR;
            
            % Battery
            QR.Battery.N_p.Value = N_p_batt;
            QR.Battery.N_s.Value = N_s_batt;
            
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

