classdef Optimization_BattProp < handle  
    properties
        quad_rotor QuadRotor

        propMassFit propMassFit
        propAeroFit propAeroFit
        
        X0 double
        lb double
        ub double
    end
    
    properties (Constant)
        X_desc = ["D","P","N_p","N_s"]
        i_X_prop = 1:2
        i_X_batt = 3:4
    end
    
    methods
        function obj = Optimization_BattProp()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', 'Battery', 'N_p', symParam('N_p',1), 'N_s', symParam('N_s',3)); % 4000mAh, 3S Default Battery, No Dynamics for now
            prop = Propeller('Name', 'Propeller', 'k_P', symParam('k_P', 0.03), 'k_T', symParam('k_T', 0.05), 'D', symParam('D', 0.1270), 'M', symParam('M', 0.008), 'J', symParam('J', 2.1075e-05));
            obj.quad_rotor = QuadRotor('Battery', batt, 'Propeller', prop);
            
            load PF_Aero.mat PF_Aero
            obj.propAeroFit = PF_Aero;
            
            load PF_Mass.mat PF_Mass
            obj.propMassFit = PF_Mass; 
            
            obj.X0 = [];
            obj.X0(obj.i_X_prop,1) = obj.propAeroFit.Boundary.X_mean;
            obj.X0(obj.i_X_batt,1) = [1; 3];
            
            lb = [];
            lb(obj.i_X_prop,1) = obj.propAeroFit.Boundary.X_lb;
            lb(obj.i_X_batt,1) = [0.1;1];
            obj.lb = lb;
            
            ub = [];
            ub(obj.i_X_prop,1) = obj.propAeroFit.Boundary.X_ub;
            ub(obj.i_X_prop(1),1) = 0.35; % Constrain max diameter of prop
            ub(obj.i_X_batt,1) = [20;12];
            obj.ub = ub;
        end

        function [opt_X, opt_flight_time, output] = Optimize(obj,r, opts)
            arguments
                obj
                r = @(t) (t>0)
                opts.SimulationBased = false
                opts.DiffMinChange = 1e-4
                opts.FlightTimeOpts cell = {}
                opts.OptimizationOpts cell = {}
            end
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp','Display', 'iter-detailed', 'PlotFcn', {@optimplotx,@optimplotfval,@optimplotfirstorderopt});
            if opts.SimulationBased
                optimopts = optimoptions(optimopts, 'DiffMinChange', opts.DiffMinChange);
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});

            [opt_X, fval, ~, output] = fmincon(@(X) -obj.flightTime(X,r,'SimulationBased', opts.SimulationBased, opts.FlightTimeOpts{:}),obj.X0, [], [], [], [], obj.lb, obj.ub, @(x) nlconBoundary(obj,x), optimopts);

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
                opts.Timeout = 5
            end

           
           try
               obj.updateSymParamVals(X);
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
        
        function [c,ceq] = nlconBoundary(obj,x)
            c = distToBoundary(obj.propAeroFit.Boundary, x(obj.i_X_prop));
            ceq = [];
        end
        
        function sym_param_vals = updateSymParamVals(obj,X)    
            %X_prop = [D;P]
            X_prop = X(obj.i_X_prop);
            
            %X_batt = [N_p; N_s]
            X_batt = X(obj.i_X_batt);
            
           D = X_prop(1);
           [k_P, k_T] = calcPropCoeffs(obj.propAeroFit, X_prop);
           [M,J] = calcMassProps(obj.propMassFit, D);
           
           %   sym_params:   D,J,M,N_p,N_s,k_P,k_T
           sym_param_vals = [D;J;M;X_batt;k_P;k_T];
           obj.quad_rotor.updateSymParamVals(sym_param_vals);
        end
    end
end

