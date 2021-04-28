classdef Optimization_General < handle  
    properties
        quad_rotor QuadRotor

        propMassFit propMassFit
        propAeroFit propAeroFit
        motorFit motorFit
        
        X0 double
        lb double
        ub double
    end
    
    properties (Constant)
        X_desc = ["D","P","N_p","N_s","kV","Rm"]
        i_X_prop = 1:2
        i_X_batt = 3:4
        i_X_motor = 5:6
    end
    
    methods
        function obj = Optimization_General()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', 'Battery', 'N_p', symParam('N_p_batt',1), 'N_s', symParam('N_s_batt',3)); % 4000mAh, 3S Default Battery, No Dynamics
            prop = Propeller('Name', 'Propeller', 'k_P', symParam('k_P_prop', 0.03), 'k_T', symParam('k_T_prop', 0.05), 'D', symParam('D_prop', 0.1270), 'M', symParam('M_prop', 0.008), 'J', symParam('J_prop', 2.1075e-05));
            motor = PMSMMotor('Name','Motor', 'M', symParam('M_motor',0.04), 'J', symParam('J_motor', 6.5e-6), 'K_t', symParam('K_t_motor', 0.00255), 'R_1', symParam('Rm_motor',0.117));
            obj.quad_rotor = QuadRotor('Battery', batt, 'Propeller', prop, 'PMSMMotor', motor);
            
            load PF_Aero.mat PF_Aero
            obj.propAeroFit = PF_Aero;
            
            load PF_Mass.mat PF_Mass
            obj.propMassFit = PF_Mass;
            
            load MF.mat MF
            obj.motorFit = MF;
            
            obj.X0 = [];
            obj.X0(obj.i_X_prop,1) = obj.propAeroFit.Boundary.X_mean;
            obj.X0(obj.i_X_batt,1) = [1; 3];
            obj.X0(obj.i_X_motor,1) = obj.motorFit.Boundary.X_mean;
            
            
            lb = [];
            lb(obj.i_X_prop,1) = obj.propAeroFit.Boundary.X_lb;
            lb(obj.i_X_batt,1) = [0.1;1];
            lb(obj.i_X_motor,1) = obj.motorFit.Boundary.X_lb;
            obj.lb = lb;
            
            ub = [];
            ub(obj.i_X_prop,1) = obj.propAeroFit.Boundary.X_ub;
            ub(obj.i_X_prop(1),1) = 0.35; % Constrain max diameter of prop to avoid fit artifact
            ub(obj.i_X_batt,1) = [20;12];
            ub(obj.i_X_motor,1) = obj.motorFit.Boundary.X_ub;
            obj.ub = ub;
        end

        function [opt_X, opt_flight_time, output] = Optimize(obj,r, opts)
            arguments
                obj
                r = @(t) (t>0)
                opts.SimulationBased = false
                opts.DiffMinChange = 1e-4
                opts.FlightTimeOpts cell = {'SimulationOpts',{'RelTol', 1e-6}}
                opts.OptimizationOpts cell = {}
            end
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp','Display', 'iter-detailed', 'PlotFcn', {@optimplotx,@optimplotfval,@optimplotfirstorderopt});
            if opts.SimulationBased
                optimopts = optimoptions(optimopts, 'DiffMinChange', opts.DiffMinChange);
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});

            [opt_X, fval, ~, output] = fmincon(@(X) -obj.flightTime(X,r,'SimulationBased', opts.SimulationBased,opts.FlightTimeOpts{:}),obj.X0, [], [], [], [], obj.lb, obj.ub, @(x) nlcon(obj,x), optimopts);

            opt_flight_time = -fval;
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
                opts.Timeout = 60
                opts.SimulationOpts = {'RelTol',1e-6}
            end

           
           try
               obj.updateSymParamVals(X);
               if opts.SimulationBased
                   if opts.RecomputeControlGains
                       obj.quad_rotor.calcControllerGains;
                   end
                   ft = obj.quad_rotor.flightTime(r,'SimulationBased',true, 'InterpolateTime', opts.InterpolateTime, 'Timeout', opts.Timeout, 'SimulationOpts', opts.SimulationOpts);
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
        
        function [c,ceq] = nlcon(obj,x)
            c_1 = distToBoundary(obj.propAeroFit.Boundary, x(obj.i_X_prop));
            c_2 = distToBoundary(obj.motorFit.Boundary, x(obj.i_X_motor));
            c = [c_1;c_2];
            ceq = [];
        end
        
        function sym_param_vals = updateSymParamVals(obj,X)
            %X_prop = [D;P]
            X_prop = X(obj.i_X_prop);
            D_prop = X_prop(1);
            [k_P_prop, k_T_prop] = calcPropCoeffs(obj.propAeroFit, X_prop);
            [M_prop,J_prop] = calcMassProps(obj.propMassFit, D_prop);
                   
            %X_batt = [N_p; N_s]
            X_batt = X(obj.i_X_batt);
            N_p_batt = X_batt(1);
            N_s_batt = X_batt(2);
            
            %X_motor = [kV; Rm]
            X_motor = X(obj.i_X_motor);
            Rm_motor = X_motor(2);
            [K_t_motor, M_motor, J_motor] = calcMotorProps(obj.motorFit, X_motor);
            
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
            
            sym_param_vals = [D_prop; J_motor; J_prop; K_t_motor; M_motor; M_prop; N_p_batt; N_s_batt; Rm_motor; k_P_prop; k_T_prop];
            obj.quad_rotor.updateSymParamVals(sym_param_vals);
        end
    end
end

