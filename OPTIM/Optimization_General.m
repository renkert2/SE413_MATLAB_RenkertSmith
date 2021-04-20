classdef Optimization_General < handle  
    properties
        quad_rotor QuadRotor

        propMassFit propMassFit
        propAeroFit propAeroFit
        motorFit motorFit
        
        OptiVars (:,1) optiVar
    end
    
    methods
        function obj = Optimization_General()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', 'Battery',...
                'N_p', compParam('N_p',1,'AutoRename', true, 'Tunable', true),...
                'N_s', compParam('N_s',3, 'AutoRename', true, 'Tunable', true)); % 4000mAh, 3S Default Battery, No Dynamics
            prop = Propeller('Name', 'Propeller',...
                'k_P', compParam('k_P',  0.0411, 'AutoRename', true, 'Tunable', true),...
                'k_T', compParam('k_T', 0.0819, 'AutoRename', true, 'Tunable', true),...
                'D', compParam('D', 0.1780, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),...
                'P', compParam('P', 0.0673, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),...
                'M', extrinsicProp('Mass', 0.008, 'AutoRename',true,'Tunable',true, 'Unit', "kg"),...
                'J', compParam('J', 2.1075e-05, 'AutoRename', true, 'Tunable',true, 'Unit', "kg*m^2"));
            motor = PMSMMotor('Name','Motor',...
                'M', extrinsicProp('Mass',0.04, 'AutoRename', true, 'Tunable', true, 'Unit', "kg"),...
                'J', compParam('J', 6.5e-6, 'AutoRename', true, 'Tunable', true, 'Unit', "kg*m^2"),...
                'D', compParam('D', 0.05, 'AutoRename', true, 'Tunable', true, 'Unit', "m"),...
                'K_t', compParam('K_t', 0.00255, 'AutoRename', true, 'Tunable', true, 'Unit', "N*m/A"),...
                'Rm', compParam('Rm',0.117, 'AutoRename', true, 'Tunable', true, 'Unit', "Ohm"));
            obj.quad_rotor = QuadRotor('Battery', batt, 'Propeller', prop, 'PMSMMotor', motor);
            
            load PF_Aero.mat PF_Aero
            obj.propAeroFit = PF_Aero;
            
            load PF_Mass.mat PF_Mass
            obj.propMassFit = PF_Mass;
            
            load MF_KDE.mat MF_KDE
            obj.motorFit = MF_KDE;
            
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
            
            OV(5) = optiVar("kV", obj.motorFit.Boundary.X_mean(1), obj.motorFit.Boundary.X_lb(1), obj.motorFit.Boundary.X_ub(1));
            OV(5).Description = "Speed Constant";
            OV(5).Unit = "RPM/V";
            OV(5).Parent = motor;
            
            OV(6) = optiVar("Rm", obj.motorFit.Boundary.X_mean(2), obj.motorFit.Boundary.X_lb(2), obj.motorFit.Boundary.X_ub(2));
            OV(6).Description = "Phase Resistance";
            OV(6).Unit = "Ohm";
            OV(6).Parent = motor;

            obj.OptiVars = OV';
        end

        function [X_opt_s, opt_flight_time, output] = Optimize(obj,r, opts)
            arguments
                obj
                r = @(t) (t>0)
                opts.SimulationBased = false
                opts.DiffMinChange = 1e-4
                opts.FlightTimeOpts cell = {}
                opts.OptimizationOpts cell = {}
                opts.OptimizationOutput logical = true
            end
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp');
            if opts.SimulationBased
                optimopts = optimoptions(optimopts, 'DiffMinChange', opts.DiffMinChange);
            end
            if opts.OptimizationOutput
                optimopts = optimoptions(optimopts, 'Display', 'iter-detailed', 'PlotFcn', {@optimplotx,@optimplotfval,@optimplotfirstorderopt});
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});

            [X_opt_s, fval, ~, output] = fmincon(@(X_s) -obj.flightTime(XAll(obj.OptiVars,X_s),r,'SimulationBased', opts.SimulationBased, opts.FlightTimeOpts{:}),X0(obj.OptiVars), [], [], [], [], LB(obj.OptiVars), UB(obj.OptiVars), @(X_s) nlcon(XAll(obj.OptiVars,X_s)), optimopts);
            opt_flight_time = -fval;
            
            % Set Current Values to Optimal Value in OptiVars
            setVals(obj.OptiVars, X_opt_s);
            
            % Ensure the QuadRotor gets updated to the correct sym param vals
            obj.updateParamVals(XAll(obj.OptiVars));
            
            % Update QuadRotor's Flight Time to the optimal value
            obj.quad_rotor.flight_time = opt_flight_time;
            
            function [c,ceq] = nlcon(x)
                c_1 = distToBoundary(obj.propAeroFit.Boundary, x(find(obj.OptiVars, ["D", "P"])));
                c_2 = distToBoundary(obj.motorFit.Boundary, x(find(obj.OptiVars, ["kV", "Rm"])));
                c = [c_1;c_2];
                ceq = [];
            end
        
        end
        
        function ft = flightTime(obj,X,r,opts)
            arguments
                obj
                X
                r = @(t) (t>0)
                opts.SimulationBased = false
                opts.RecomputeControlGains = true
                opts.InterpolateTime = false
                opts.Scaled = false
                opts.ScalingFactor = 1500
                opts.Timeout = 5
            end
            try
                obj.updateParamVals(X);
                if opts.SimulationBased
                    if opts.RecomputeControlGains
                        obj.quad_rotor.calcControllerGains;
                    end
                    ft = obj.quad_rotor.flightTime(r,'SimulationBased',true, 'InterpolateTime', opts.InterpolateTime, 'Timeout', opts.Timeout);
                else
                    ft = obj.quad_rotor.flightTime('SimulationBased',false);
                end
            catch
                ft = NaN;
            end
            
            if opts.Scaled
                ft = ft/opts.ScalingFactor;
            end
        end
        
        function [X,ft,X_opt,ft_opt] = sweep(obj, vars, n)
            N_vars = numel(vars);
            assert(N_vars <=2 && N_vars ~= 0, "Choose 1 or 2 optimization variables for sweep")
            if ~isa(vars, 'optiVar')
                try
                    vars = string(vars);
                    vars = obj.OptiVars.get(vars);
                catch
                    error('vars argument must be optiVar objects or convertable to strings');
                end
            end
            
            obj.OptiVars.reset();
            % Obtain baseline optimal point
            [~,ft_opt] = obj.Optimize('OptimizationOutput', false, 'OptimizationOpts', {'Display', 'none'});
            X_opt = vertcat(vars.Value);
            
            for i = 1:N_vars
                % Each variable in the sweep is fixed at a point and is 
                % no longer optimized
                vars(i).Enabled = false;
            end
            
           switch N_vars
               case 1
                   X = linspace(vars, n);
                   ft = NaN(size(X));
                   for i = 1:numel(X)
                       vars.Value = X(i);
                       ft(i) = optiWrapper();
                   end
                   
                   figure
                   plot(X,ft)
                   hold on
                   plot(X_opt, ft_opt, '.r', 'MarkerSize', 20)
                   hold off
               case 2
                   x = linspace(vars(1),n);
                   y = linspace(vars(2),n);
         
                   ft = NaN(numel(y), numel(x));
                   for i = 1:numel(x)
                       for j = 1:numel(y)
                           vars(1).Value = x(i);
                           vars(2).Value = y(j);
                           ft(j,i) = optiWrapper();
                       end
                   end
                   
                   [X(:,:,1), X(:,:,2)] = meshgrid(x,y);
                   figure
                   surf(X(:,:,1), X(:,:,2),ft);
                   hold on
                   plot3(X_opt(1), X_opt(2), ft_opt, '.r', 'MarkerSize', 20);
                   hold off
           end
           
           % Format Figure
           title("Carpet Plot")
           xlabel(latex(vars(1)), 'Interpreter', 'latex')
           if N_vars == 2
               ylabel(latex(vars(2)), 'Interpreter', 'latex')
               zlabel('Flight Time (s)', 'Interpreter', 'latex')
           else
               ylabel('Flight Time (s)', 'Interpreter', 'latex')
           end
           
           % Clean Up
           obj.OptiVars.reset();
           
            function ft = optiWrapper()
                try
                    [~,ft] = obj.Optimize('OptimizationOutput', false);
                catch
                    ft = NaN;
                end
            end
        end
        
        function resetParamVals(obj)
           obj.updateParamVals(vertcat(obj.OptiVars.x0)); 
        end
        
        function updateParamVals(obj,X)
            %X_prop = [D;P]
            X_prop = X(find(obj.OptiVars, ["D", "P"]));
            D_prop = X_prop(1);
            [k_P_prop, k_T_prop] = calcPropCoeffs(obj.propAeroFit, X_prop);
            [M_prop,J_prop] = calcMassProps(obj.propMassFit, D_prop);
                   
            %X_batt = [N_p; N_s]
            X_batt = X(find(obj.OptiVars, ["N_p", "N_s"]));
            N_p_batt = X_batt(1);
            N_s_batt = X_batt(2);
            
            %X_motor = [kV; Rm]
            X_motor = X(find(obj.OptiVars, ["kV", "Rm"]));
            Rm_motor = X_motor(2);
            [K_t_motor, M_motor, J_motor, D_motor] = calcMotorProps(obj.motorFit, X_motor);
            
            % sym_params:
            %    D_prop
            %    J_motor
            %    J_prop
            %    K_t_motor
            %    M_motor
            %    M_prop
            %    N_p_batt
            %    N_s_batt
            %    Rm_motor
            %    k_P_prop
            %    k_T_prop
            QR = obj.quad_rotor;
            
            % Battery
            QR.Battery.N_p.Value = N_p_batt;
            QR.Battery.N_s.Value = N_s_batt;
            
            % Prop
            QR.Propeller.D.Value = D_prop;
            QR.Propeller.J.Value = J_prop;
            QR.Propeller.M.Value = M_prop;
            QR.Propeller.k_P.Value = k_P_prop;
            QR.Propeller.k_T.Value = k_T_prop;
            
            % Motor
            QR.Motor.J.Value = J_motor;
            QR.Motor.K_t.Value = K_t_motor;
            QR.Motor.M.Value = M_motor;
            QR.Motor.Rm.Value = Rm_motor;
            QR.Motor.D.Value = D_motor;

            obj.quad_rotor.update();
        end
    end
end

