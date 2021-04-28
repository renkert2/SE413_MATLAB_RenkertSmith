classdef QuadRotor < System
    properties (SetAccess = private)
        SymVars SymVars
        
        SimpleModel Model
        
        % SymQuantities - Properties that are functions of the QuadRotor
        % Parameters
        Mass function_handle
        BattCap function_handle
        V_OCV_pack function_handle % Open circuit voltage of the battery pack; function of Q. 
        HoverThrust function_handle % Thrust required to hover
        HoverSpeed function_handle  % Speed required to hover
        RotorSpeed function_handle % Calculates rotor speed as a function of total required thrust
        
        % Parameter-dependent properties
        SS_QAve QRState % Steady State at average battery voltage 
    end
    
    properties
        flight_time double % Expensive calculation is cached
    end
    
    % Easier Access to Components
    properties (SetAccess = private)
        Frame Frame
        Battery Battery
        DCBus DCBus_CurrentEquivalence
        Inverter PMSMInverter
        Motor PMSMMotor
        Propeller Propeller
    end
    
    properties (Dependent)
        PerformanceData
        DesignData
    end
    
    properties (Hidden)
        SteadyState_func function_handle % Used in solved for 0, i.e. solve(obj.SteadyState_func == 0)
        SteadyStateIO_func function_handle % Solved for 0 in calcSteadyStateIO
    end

    methods
        function obj = QuadRotor(p)
            arguments
                p.Frame Frame = Frame('Name', "Frame");
                p.Battery Battery = Battery('Name', "Battery");
                p.DCBus DCBus_CurrentEquivalence = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4);
                p.PMSMInverter PMSMInverter = PMSMInverter('Name', "Inverter");
                p.PMSMMotor PMSMMotor = PMSMMotor('Name', "Motor");
                p.Propeller Propeller = Propeller('Name', "Propeller");
            end
           
            pmsminverters = Replicate([p.PMSMInverter], 4);
            pmsminverters = pmsminverters{1};
            
            motorprop = MotorProp('PMSMMotor', p.PMSMMotor, 'Propeller', p.Propeller);
            motorprops = Replicate(motorprop, 4, 'RedefineChildren', false);
            motorprops = vertcat(motorprops{:});
           
            
            % I would love for this to work at some point.
            % motorprop = MotorProp('PMSMMotor', p.PMSMMotor, 'Propeller', p.Propeller);
            % pmsminverters = repmat(p.PMSMInverter, 1, 4);
            % motorprops = repmat(motorprop, 1, 4);
            
            Components = [p.Frame; p.Battery; p.DCBus; pmsminverters; motorprops];
            
            ConnectP = {[p.Battery.Ports(1) p.DCBus.Ports(1)]};
            
            for i = 1:4
                ConnectP{end+1,1} = [pmsminverters(i).Ports(1),p.DCBus.Ports(1+i)];
                ConnectP{end+1,1} = [pmsminverters(i).Ports(2), motorprops(i).Ports(1)];
            end
            
            obj = obj@System("Quad Rotor", Components, ConnectP);
            obj.Frame = p.Frame;
            obj.Battery = p.Battery;
            obj.DCBus = p.DCBus;
            obj.Inverter = p.PMSMInverter;
            obj.Motor = p.PMSMMotor;
            obj.Propeller = p.Propeller;
            
            warning('off', 'Control:combination:connect10') % Annoying message from calcControllerGains
            init_post(obj);
        end
        
        function init_post(obj)
            createModel(obj);
            setParamQuantities(obj);
            setSimpleModel(obj);
            setSteadyStateFunc(obj);
            setSteadyStateIOFunc(obj);
            update(obj);
        end

        function update(obj)
            obj.SS_QAve = calcSteadyState(obj);
            obj.flight_time = [];
        end
        
        function setParamQuantities(obj)
            p = obj.Params;
            PF = @(s) matlabFunction(p,s); % Wrapper function for brevity
            
            % Battery Capacity
            batt = obj.Battery;
            obj.BattCap = PF(batt.Capacity); % A*s
            
            % Battery Pack V_OCV
            obj.V_OCV_pack = matlabFunction(p, batt.V_OCV_pack, {sym('q')});
            
            % Mass
            exps = obj.Params.extrinsicProps;
            masses = getProp(exps, 'Mass');
            mass = masses(end);
            obj.Mass = PF(mass(end));
            
            % Hover Thrust
            hover_thrust = 9.81*mass;
            obj.HoverThrust = PF(hover_thrust); % Total Thrust required to hover

            % Hover Speed
            prop = obj.Propeller;
            hover_speed = prop.calcSpeed(hover_thrust/4);
            obj.HoverSpeed = PF(hover_speed);
            
            % Rotor Speed Function 
            % - Calculates required rotor speed as a function of total
            % thrust
            T_reqd = sym("T_reqd");
            rotor_speed = prop.calcSpeed(T_reqd/4);
            obj.RotorSpeed = matlabFunction(p, rotor_speed, {T_reqd});
        end
        
        function setSimpleModel(obj)
            simple_model = Model();
            
            % STATES
            %dyn_eqns dot(x) = dyn_eqns(x)
            %1.         x1       "Battery"        "Battery SOC"           Abstract
            %2.         x2       "Motor"        "Inductance (i_q)"      Current
            %3.         x3       "Motor"        "Inertia (omega_m)"     AngularVelocity
            
            states = [1,2,3];
            simple_model.StateDescriptions = ["Battery SOC"; "Motor Current"; "Rotor Speed"];
            simple_model.Nx = numel(states);
            
            % INPUTS
            simple_model.Nu = 1;
            simple_model.InputDescriptions = ["Inverter Input"];
            
            
            % OUTPUTS:
            %1-3.  States 1-3
            %4.  Bus Voltage
            %5.  Bus Current
            %6.  Inverter Current (DC)
            %7.  Inverter Voltage (Q)
            %8. Torque
            %9. Thrust - Total
  
            outs = [states 10 11 12 13 21 20];
            simple_model.Ny = numel(outs);
            simple_model.OutputDescriptions = [simple_model.StateDescriptions; "Bus Voltage"; "Bus Current"; "Inverter Current (DC)"; "Inverter Voltage (q)"; "Torque"; "Total Thrust"];
            
            obj.SymVars = SymVars('Nx', simple_model.Nx, 'Nd', simple_model.Nd, 'Nu', simple_model.Nu);
            
            digits 8 % Set VPA Digits to 8
            
            % Set f_sym
            dyn_eqns = vpa(obj.Model.f_sym(states));
            simple_model.f_sym = combine(vpa(obj.convertSyms(dyn_eqns))); % combine makes sym expression a bit simpler
            
            % Set g_sym
            g_sym_mod = obj.convertSyms(vpa(obj.Model.g_sym(outs)));
            g_sym_mod(end) = g_sym_mod(end)*4; % Convert Thrust per Propeller to Total Thrust
            simple_model.g_sym = combine(g_sym_mod); % combine makes sym expression a bit simpler
            
            simple_model.Params = obj.Params;
            simple_model.SymVars = obj.SymVars;
            
            simple_model.init();
            
            obj.SimpleModel = simple_model;
        end
        
        function setSteadyStateFunc(obj)
            solve_vars = [sym('u1'); sym('x2')];
            subs_vars = [sym('x1'); sym('x3')];
            
            args = {solve_vars, subs_vars};
            
            obj.SteadyState_func = matlabFunction(obj.Params, obj.SimpleModel.f_sym(2:end), args);
        end
        
        function setSteadyStateIOFunc(obj)
            solve_vars = [sym('x2'); sym('x3')];
            subs_vars = [sym('u1'); sym('x1')];
            
            args = {solve_vars, subs_vars};
            
            obj.SteadyStateIO_func = matlabFunction(obj.Params, obj.SimpleModel.f_sym(2:end),args);
        end
        
        function qrss = calcSteadyState(obj, q_bar, T_reqd, opts)
            % Calculates steady-state values of the powertrain model at thrust T_reqd, returns QRSteadyState object
            % Default value for q_bar is the average battery soc
            % Default value fot T_reqd is the HoverThrust
            
            arguments
                obj
                q_bar double = [];
                T_reqd double = [];
                opts.SolverOpts struct = optimset('Display','off');
            end
            
            if isempty(q_bar)
                q_bar = obj.Battery.Averaged_SOC;
            end
            
            if isempty(T_reqd)
                rotor_speed = obj.HoverSpeed();
            else
                rotor_speed = obj.RotorSpeed(T_reqd);
            end
            
            
            x0 = [0.5; 1];
            [x_sol, ~, exit_flag] = fsolve(@(x) obj.SteadyState_func(x,[q_bar;rotor_speed]), x0, opts.SolverOpts);
            if exit_flag <= 0
                error('No solution found');
            elseif x_sol(1) > 1
                error('No valid solution.  Required input exceeds 1');
            elseif x_sol(1) < 0
                error('No valid solution.  Required input must be positive');
            end
            
            qrss = QRState();
            qrss.q = q_bar;
            qrss.x = [q_bar; x_sol(2:end); rotor_speed];
            qrss.u = x_sol(1);
            qrss.y = obj.SimpleModel.CalcG(qrss.x, qrss.u, []);
            qrss.BatteryOCV = obj.V_OCV_pack(q_bar);
        end
        
        function qrss = calcSteadyStateIO(obj, u, q, opts)
            % Calculates steady-state thrust given input u, battery SOC q
            % Default value for q is average battery soc
            
            arguments
                obj
                u
                q double = []
                opts.SolverOpts = optimset('Display','off');
            end
            
            if isempty(q)
                q = getComponents(obj,'Battery').Averaged_SOC;
            end
            
            hover_speed = obj.HoverSpeed(); % Just used for initialization.
            
            x0 = [1; hover_speed];
            [x_sol, ~, exit_flag] = fsolve(@(x) obj.SteadyStateIO_func(x,[u;q]), x0, opts.SolverOpts);
            if exit_flag <= 0
                error('No solution found');
            end
            
            x_sol = [q; x_sol];
            u_sol = u;
            y_sol = obj.SimpleModel.CalcG(x_sol, u_sol, []);
            
            qrss = QRState();
            qrss.q = q;
            qrss.x = x_sol;
            qrss.u = u_sol;
            qrss.y = y_sol;
            qrss.BatteryOCV = obj.V_OCV_pack(q);
        end
        
        function u_bar_func = calcSteadyStateInputFunc(obj, opts)
            arguments
                obj
                opts.Resolution double = 0.01
            end
            
            solver_opts = optimset('Display','off');
            
            q_vals = 0:opts.Resolution:1;
            u0_vals = zeros(size(q_vals));
            for i = 1:numel(q_vals)
                qrss = calcSteadyState(obj, q_vals(i), 'SolverOpts', solver_opts);
                u0_vals(i) = qrss.u;
            end
            
            u_bar_func = @(q) interp1(q_vals, u0_vals, q, 'pchip');
        end
        
        function tr = calcThrustRatio(obj)
            qrsio = calcSteadyStateIO(obj, 1);
            T_max = qrsio.TotalThrust;
            T_hover = obj.HoverThrust();
            
            tr = T_max / T_hover;
        end

        function [flight_time] = flightTime(obj)
            arguments
                obj
            end
            cap = obj.BattCap(); % A*s
            ave_current = obj.SS_QAve.y(5);
            flight_time = cap/ave_current;
            
            obj.flight_time = flight_time;
        end
        
        function pd = get.PerformanceData(obj)
            pd = PerformanceData();
            pd.FlightTime = obj.flight_time;
            pd.ThrustRatio = calcThrustRatio(obj);
            pd.SteadyState = obj.SS_QAve;
        end
        
        function dd = get.DesignData(obj)
            dd = exportStruct(obj.Params);
        end
    end
    
    %% Helper Functions
    methods (Access = private)
        function syms_out = convertSyms(obj, syms_in)
            syms_out = convertToBalanced(obj,syms_in);
            syms_out = convertStates(obj,syms_out);
        end
        
        function sym_bal = convertToBalanced(obj, sym_unbal)
            % Replaces independent rotor variables with the same variable so that each
            % rotor has the same symbolic variable
            persistent from to
            if isempty(from) || isempty(to)
                from = {};
                to = {};
                
                from{1} = [sym('u2'), sym('u3'), sym('u4')];
                to{1} = repmat(sym('u1'),1,3);
                
                from{2} = [sym('x4'), sym('x6'), sym('x8')];
                to{2} = repmat(sym('x2'),1,3);
                
                from{3} = [sym('x5'), sym('x7'), sym('x9')];
                to{3} = repmat(sym('x3'),1,3);
            end
            
            sym_bal = subs(sym_unbal,[from{:}], [to{:}]);
        end
        
        function sym_out = convertStates(obj, sym_in)
            from = [sym('x1'); sym('x2'); sym('x3')];
            to = obj.SymVars.x;
            sym_out = subs(sym_in, from,to);
        end
    end
end