classdef QuadRotor < System
    properties
        StandardEmptyWeight = extrinsicProp('Mass', 0.284 - 0.080 - 4*0.008) % Mass Not Including Battery and Propellers.  Those are added later
    end
    
    properties (SetAccess = private)
        BattCap
        Mass
        HoverThrust % Thrust required to hover
        HoverSpeed % Speed required to hover
        
        SimpleModel Model
        LinearDynamicModel LinearModel
        
        SymVars SymVars
    end
    
    properties (Hidden)
        SteadyState_func function_handle
        HoverThrust_func function_handle
        HoverSpeed_func function_handle
        Mass_func function_handle
        BattCap_func function_handle
    end
    
    properties (SetAccess = private)
        sym_param_vals double
        q_bar double % Battery SOC at which ss vals are calculated
        x_bar double
        u_bar double
        y_bar double
    end
    
    methods
        function obj = QuadRotor(p)
            arguments
                p.Battery Battery = Battery('Name', "Battery");
                p.DCBus DCBus_CurrentEquivalence = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4);
                p.PMSMInverter PMSMInverter = PMSMInverter('Name', "Inverter");
                p.PMSMMotor PMSMMotor = PMSMMotor('Name', "Motor");
                p.ShaftCoupler ShaftCoupler = ShaftCoupler('Name', "Shaft");
                p.Propeller Propeller = Propeller('Name', "Propeller");
            end
            
            replicated_comps = Replicate([p.PMSMInverter, p.PMSMMotor, p.ShaftCoupler, p.Propeller], 4);
            pmsminverters = replicated_comps{1};
            pmsmmotors = replicated_comps{2};
            shaftcouplers = replicated_comps{3};
            propellers = replicated_comps{4};
            
            components = [p.Battery, p.DCBus, replicated_comps{:}];
            
            ConnectP = {[p.Battery.Ports(1) p.DCBus.Ports(1)]};
            
            for i = 1:4
                ConnectP{end+1,1} = [pmsminverters(i).Ports(1),p.DCBus.Ports(1+i)];
                ConnectP{end+1,1} = [pmsminverters(i).Ports(2), pmsmmotors(i).Ports(1)];
                ConnectP{end+1,1} = [pmsmmotors(i).Ports(2), shaftcouplers(i).Ports(1)];
                ConnectP{end+1,1} = [shaftcouplers(i).Ports(2), propellers(i).Ports(1)];
            end
            
            obj = obj@System(components, ConnectP);
            obj.extrinsicProps = Combine([obj.extrinsicProps,obj.StandardEmptyWeight]);
            init_post(obj);
        end
        
        function init_post(obj)
            obj.createModel();
            setBattCap(obj);
            setMass(obj);
            setHoverThrust(obj);
            setHoverSpeed(obj);
            setSimpleModel(obj);
            setLinearDynamicModel(obj);
            setSteadyStateFunc(obj);
            updateSymParamVals(obj, obj.Graph.SymParams.Vals);
        end
        
        function updateSymParamVals(obj, sym_param_vals)
            obj.sym_param_vals = sym_param_vals;
            obj.calcSteadyState('StoreSolution',true);
        end
        
        function setBattCap(obj)
            batt = obj.Components(1);
            obj.BattCap = batt.Capacity; % A*s
            
            if ~isempty(obj.Graph.SymParams)
                obj.BattCap_func = matlabFunction(obj.BattCap, 'Vars', {obj.Graph.SymParams.Syms});
            else
                obj.BattCap_func = @(x) obj.BattCap;
            end
        end
            
        function setMass(obj)
            obj.Mass = getProp(obj.extrinsicProps, 'Mass');
            
            if ~isempty(obj.Graph.SymParams)
                obj.Mass_func = matlabFunction(obj.Mass, 'Vars', {obj.Graph.SymParams.Syms});
            else
                obj.Mass_func = @(x) obj.Mass;
            end
        end
        
        function setHoverThrust(obj)
            total_mass = obj.Mass;
            obj.HoverThrust = (9.81*total_mass); % Total Thrust required to hover
            
            if ~isempty(obj.Graph.SymParams)
                obj.HoverThrust_func = matlabFunction(obj.HoverThrust, 'Vars', {obj.Graph.SymParams.Syms});
            else
                obj.HoverThrust_func = @(x) obj.HoverThrust;
            end
        end
        
        function setHoverSpeed(obj)
            if isempty(obj.HoverThrust)
                obj.setHoverThrust;
            end
            prop_i = find(arrayfun(@(x) isa(x,'Propeller'), obj.Components),1);
            prop = obj.Components(prop_i);
            obj.HoverSpeed = prop.calcSpeed(obj.HoverThrust/4);
            
            if ~isempty(obj.Graph.SymParams)
                obj.HoverSpeed_func = matlabFunction(obj.HoverSpeed, 'Vars', {obj.Graph.SymParams.Syms});
            else
                obj.HoverSpeed_func = @(x) obj.HoverSpeed;
            end
        end
        
        function setSimpleModel(obj)
            simple_model = Model();
            
            % STATES
            %dyn_eqns dot(x) = dyn_eqns(x)
            %1.         x1       "Battery"        "Battery SOC"           Abstract
            %2.         x2       "Motor"        "Inductance (i_q)"      Current
            %3.         x3       "Motor"        "Inertia (omega_m)"     AngularVelocity
            %4.         x10      "Shaft"        "Torque (T)"            Torque
            %5.         x14      "Propeller"    "Inertia (omega)"       AngularVelocity
            states = [1,2,3,10,14];
            simple_model.StateDescriptions = ["Battery SOC"; "Motor Current"; "Motor Velocity"; "Shaft Torque"; "Propeller Speed"];
            simple_model.Nx = numel(states);
            
            % INPUTS
            simple_model.Nu = 1;
            simple_model.InputDescriptions = ["Inverter Input"];

            
            % OUTPUTS:
            %1-5.  States 1-5
            %6.  Bus Voltage
            %7.  Bus Current
            %8.  Thrust - Total
            outs = [states 18 19 28];
            simple_model.Ny = numel(outs);
            simple_model.OutputDescriptions = [simple_model.StateDescriptions; "Bus Voltage"; "Bus Current"; "Total Thrust"];
            
            obj.SymVars = SymVars('Nx', simple_model.Nx, 'Nd', simple_model.Nd, 'Nu', simple_model.Nu);  
            
            digits 8 % Set VPA Digits to 8
            
            % Set f_sym
            dyn_eqns = vpa(obj.Model.f_sym(states));
            simple_model.f_sym = combine(vpa(obj.convertSyms(dyn_eqns))); % combine makes sym expression a bit simpler
            
            % Set g_sym
            g_sym_mod = obj.convertSyms(vpa(obj.Model.g_sym(outs)));
            g_sym_mod(end) = g_sym_mod(end)*4; % Convert Thrust per Propeller to Total Thrust
            simple_model.g_sym = combine(g_sym_mod); % combine makes sym expression a bit simpler
            
            simple_model.SymParams = obj.Graph.SymParams;
            simple_model.SymVars = obj.SymVars;
            
            simple_model.init();
            
            obj.SimpleModel = simple_model;  
        end
        
        function setLinearDynamicModel(obj)
            u_mod = obj.SymVars.u;
            
            % Cut Battery Dynamics out of Dynamic Model
            x_mod = obj.SymVars.x(2:end);
            f_sym_mod = obj.SimpleModel.f_sym(2:end);
            g_sym_mod = obj.SimpleModel.g_sym(end);
            
            A = jacobian(f_sym_mod, x_mod);
            B = jacobian(f_sym_mod, u_mod);
            
            C = jacobian(g_sym_mod, x_mod);
            D = jacobian(g_sym_mod, u_mod);
            
            lm = LinearModel();
            lm.Nx = 4;
            lm.Nu = 1;
            lm.Nd = 0;
            lm.Ny = 1;
            lm.StateDescriptions = obj.SimpleModel.StateDescriptions(2:end);
            lm.InputDescriptions = obj.SimpleModel.InputDescriptions;
            lm.OutputDescriptions = obj.SimpleModel.OutputDescriptions(end);
            sp = obj.SimpleModel.SymParams;
            sp = sp.prepend(symParam('x1',1));
            lm.SymParams = sp;
            sv = obj.SimpleModel.SymVars;
            sv.x = sv.x(2:end);
            lm.SymVars = sv;
            
            lm.A_sym = A;
            lm.B_sym = B;
            lm.E_sym = zeros(lm.Nx,1);
            lm.C_sym = C;
            lm.D_sym = D;
            lm.H_sym = zeros(lm.Ny,1);
            
            lm.f0 = zeros(lm.Nx,1);
            lm.g0 = zeros(lm.Ny,1);
            
            lm.init();
            obj.LinearDynamicModel = lm;
        end
        
        function setSteadyStateFunc(obj)
            solve_vars = [sym('u1'); sym('x2'); sym('x3'); sym('x4')];
            subs_vars = [sym('x1'); sym('x5')];
            sp_vars = obj.SimpleModel.SymParams.Syms;
            
            vars = {solve_vars, subs_vars, sp_vars};
            
            obj.SteadyState_func = matlabFunction(obj.SimpleModel.f_sym(2:end),'Vars',vars);
        end
        
        function [x_bar, u_bar, y_bar] = calcSteadyState(obj, q_bar, opts)
            arguments
                obj
                q_bar double = 1;
                opts.StoreSolution logical = false;
                opts.SolverOpts struct = optimset('Display','off');
            end
            
            sym_param_vals = obj.sym_param_vals;
            
            hover_speed = obj.HoverSpeed_func(sym_param_vals);
            
            x0 = [0.5; 1; hover_speed; 0.01];
            [x_sol, ~, exit_flag] = fsolve(@(x) obj.SteadyState_func(x,[q_bar;hover_speed],sym_param_vals), x0, opts.SolverOpts);
            if exit_flag <= 0
                error('No solution found');
            end            
            
            x_bar = [q_bar; x_sol(2:end); hover_speed];
            u_bar = x_sol(1);
            y_bar = obj.SimpleModel.CalcG(x_bar, u_bar, [], sym_param_vals);
            
            if opts.StoreSolution
                obj.q_bar = q_bar;
                obj.x_bar = x_bar;
                obj.u_bar = u_bar; 
                obj.y_bar = y_bar; 
            end
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
                [~,u_bar,~] = calcSteadyState(obj, q_vals(i), 'SolverOpts', solver_opts);
                u0_vals(i) = u_bar;
            end
            
            u_bar_func = @(q) interp1(q_vals, u0_vals, q, 'pchip');
        end
        
        function [A,B,C,D] = calcLinearMatrices(obj)
            [A,B,~,C,D,~] = CalcMatrices(obj.LinearDynamicModel,obj.x_bar(2:end),obj.u_bar,[],[obj.x_bar(1); obj.sym_param_vals]);
        end
        
        function [A,B,C,D] = calcBodyModel(obj)
            m = obj.Mass_func(obj.sym_param_vals);
            
            A = [0 1;0 0];
            B = [0; 1/m];
            C = [1 0];
            D = [0];
        end
        
        function K_PD = calcPDGains(obj)
            [A, B, C, D] = calcLinearMatrices(obj); % Obtain linearized model at hover
            gain = D-C*(A\B);
            dom_pole = min(abs(eigs(A)));
            PT_simple = tf(gain, [(1/dom_pole), 1]);
            
            [A,B,C,D] = calcBodyModel(obj);
            body = ss(A,B,C,D);
            
            plant = series(PT_simple,body);
            
            opts = pidtuneOptions('DesignFocus','disturbance-rejection', 'PhaseMargin', 60);
            pdController = pidtune(plant, 'PD', opts);
            K_PD = [pdController.Kp, pdController.Kd];
        end
        
        function [t_out, y_out, x_g_out, errFlag] = Simulate(obj, r, opts)
            arguments
                obj
                r function_handle = (@(t) t>=0)
                opts.ReferenceFilterTimeConstant double = 1
                opts.MaxSimTime double = 5e4
                opts.Timeout double = inf
                opts.PlotResults logical = true
            end

            % init
            model = obj.SimpleModel;
            sym_param_vals = obj.sym_param_vals;
            
            % Indices of x_g, x_c in composite x = [x_g; x_c; x_r]
            persistent i_x_g i_x_c i_x_r
            if isempty(i_x_g)
                i_x_g = 1:obj.SimpleModel.Nx;
                i_x_c = i_x_g(end) + (1:2);
                i_x_r = i_x_c(end) + 1;
            end
         
            % Controller Model
            % Computes states [y diff(y)]
            [Ac,Bc,~,~] = calcBodyModel(obj);
            f_c = @(x_c,u_c) Ac*x_c + Bc*u_c;
            
            % Controller Gains
            K_PD = calcPDGains(obj);
            
            % Nominal Thrust
            T_bar = obj.y_bar(end);
            
            % SS Input
            u_bar_func = calcSteadyStateInputFunc(obj, 'Resolution', 0.02);
            
            % Initial Values
            x_0 = zeros(i_x_c(end),1);
            x_0(i_x_g) = obj.x_bar;
            x_0(i_x_g(1)) = 1;
            x_0(i_x_c) = [0;0];
            x_0(i_x_r) = 0;
            
            % Reference Filter
            % Smooths Reference
            tau = opts.ReferenceFilterTimeConstant;
            f_r = @(x_r, u_r) -(1/tau)*x_r + u_r;
            g_r = @(x_r, u_r) [1/tau; -1/tau^2]*x_r + [0; 1/tau]*u_r;
            g_r_1 = @(x_r) 1/tau.*x_r;
            
            sim_opts = odeset('Events', @emptyBattery, 'OutputFcn', @odeFunc); 
            errFlag = 0;
            tic
            
            [t_out,x_out] = ode23tb(@(t,x) f_sys_cl(t,x), [0 opts.MaxSimTime], x_0, sim_opts);
            x_c_out = x_out(:,i_x_c);
            y_out = x_c_out(:,1);  
            x_g_out = x_out(:,i_x_g);
            x_r_out = x_out(:,i_x_r);
            
            if opts.PlotResults
                plotResults()
            end
            
            function x_dot = f_sys_cl(t,x)
                x_g = x(i_x_g);
                x_c = x(i_x_c);
                x_r = x(i_x_r);
                
                dot_x_r = f_r(x_r,r(t));
                r_PD = g_r(x_r,r(t));
                
                e = r_PD-x_c;
                u = K_PD*e + u_bar_func(x_g(1));
                u = min(max(u,0),1);
                
                dot_x_g = model.CalcF(x_g, u, [], sym_param_vals);
                
                y = model.CalcG(x_g, u, [], sym_param_vals);
                T = y(end) - T_bar;
                dot_x_c = f_c(x_c, T);
                
                x_dot = [dot_x_g; dot_x_c; dot_x_r];
            end
            function [value, isterminal, direction] = emptyBattery(~,y)
                % emptyBattery() is an event that terminates the simulation
                % when the battery SOC drains to zero.
                value = y(i_x_g(1));
                isterminal = 1;
                direction = 0;
            end
            function status = odeFunc(~,~,flag)
                status = 0;
                if isempty(flag)
                    timer = opts.Timeout - toc;
                    if timer < 0
                        warning('Solver timed out')
                        status = 1;
                        errFlag=1;
                    end
                end
            end
            function plotResults()
                figure(1)
                plot(t_out, [r(t_out), g_r_1(x_r_out),y_out]);
                title("Tracking Performance");
                legend(["Reference", "Filtered Reference", "Response"]);
                xlabel("t")
                ylabel("Height (m)")
            end
        end
        
        function [flight_time] = flightTime(obj, r, opts)
            arguments
                obj
                r function_handle = (@(t) t>=0)
                opts.InterpolateTime logical = false % This didn't help with the smoothness of the response at all.
            end
            
            [t,~,x_g,errFlag] = Simulate(obj, r, 'Timeout', 5, 'PlotResults', false);
            if errFlag
                flight_time = NaN;
            else
                if opts.InterpolateTime
                    q_ = x_g(end-1:end,1);
                    t_ = t(end-1:end);
                    a = -q_(1)/(q_(2)-q_(1));
                    flight_time = (1-a)*t_(1) + a*t_(2);
                else
                    flight_time = t(end);
                end
            end
        end
        
        function [flight_time] = flightTime_Simple(obj)
            batt = obj.Components(1);
            ave_q = batt.Averaged_SOC;
            cap = obj.BattCap_func(obj.sym_param_vals); % A*s

            [~, ~, y_bar] = calcSteadyState(obj, ave_q); % In the future, we may be able to calculate ss values w.r.t. ave_q by default and store it for each setSymParamVals.  
            ave_current = y_bar(7);
            
            flight_time = cap/ave_current;
        end
        %% Helper Functions
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
                
                from{3} = [sym('x11'), sym('x12'), sym('x13')];
                to{3} = repmat(sym('x10'),1,3);
                
                from{4} = [sym('x15'), sym('x16'), sym('x17')];
                to{4} = repmat(sym('x14'),1,3);
            end
            
            sym_bal = subs(sym_unbal,[from{:}], [to{:}]);
        end
        
        function sym_out = convertStates(obj, sym_in)
            from = [sym('x1'); sym('x2'); sym('x3'); sym('x10'); sym('x14')];
            to = obj.SymVars.x;
            sym_out = subs(sym_in, from,to);
        end
    end
end