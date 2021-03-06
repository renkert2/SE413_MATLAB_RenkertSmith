classdef QuadRotor < System
    properties
        StandardEmptyWeight = extrinsicProp('Mass', 0.284 - 0.080 - 4*0.008) % Mass Not Including Battery and Propellers.  Those are added later
    end
    
    properties (SetAccess = private)
        HoverThrust % Thrust required to hover
        HoverSpeed % Speed required to hover
        
        f_sym sym
        f_sym_
        
        g_sym sym
        g_sym_
        
        Nx double
        Ny double
        Nu double
        SymVars
        
        symLinearModel LinearModel
        
        StateTable
        OutputTable
        InputTable
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
            obj.createModel('Linearize',false);
            setHoverThrust(obj)
            setHoverSpeed(obj)
            setSym(obj)
            setTables(obj)
            setSymLinearModel(obj);
        end
        
        function setHoverThrust(obj)
            total_mass = getProp(obj.extrinsicProps, 'Mass');
            obj.HoverThrust = (9.81*total_mass); % Total Thrust required to hover
        end
        
        function setHoverSpeed(obj)
            if isempty(obj.HoverThrust)
                obj.setHoverThrust;
            end
            prop_i = find(arrayfun(@(x) isa(x,'Propeller'), obj.Components),1);
            prop = obj.Components(prop_i);
            obj.HoverSpeed = prop.calcSpeed(obj.HoverThrust/4);
        end
        
        function setSym(obj)
            % STATES
            %dyn_eqns dot(x) = dyn_eqns(x)
            %1.         x1       "Battery"        "Battery SOC"           Abstract
            %2.         x2       "Motor"        "Inductance (i_q)"      Current
            %3.         x3       "Motor"        "Inertia (omega_m)"     AngularVelocity
            %4.         x10      "Shaft"        "Torque (T)"            Torque
            %5.         x14      "Propeller"    "Inertia (omega)"       AngularVelocity
            digits 8
            states = [1,2,3,10,14];
            dyn_eqns = vpa(obj.Model.f_sym(states),8);
            
            obj.Nx = numel(states);
            obj.SymVars.x = sym('x',[obj.Nx 1]);
            obj.f_sym = combine(vpa(obj.convertSyms(dyn_eqns)));
            
            % INPUTS
            obj.Nu = 1;
            obj.SymVars.u = sym('u1');
            
            % OUTPUTS:
            %1-5.  States 1-5
            %6.  Bus Voltage
            %7.  Bus Current
            %8.  Thrust - Total
            outs = [states 18 19 28];
            obj.Ny = numel(outs);
            obj.SymVars.y = sym('y', [obj.Ny 1]);
            g_sym_mod = obj.convertSyms(vpa(obj.Model.g_sym(outs)));
            g_sym_mod(end) = g_sym_mod(end)*4; % Convert Thrust per Propeller to Total Thrust
            obj.g_sym = combine(g_sym_mod);
            
            % MATLAB Funcs
            if ~isempty(obj.Model.Sym
            obj.f_sym_ = matlabFunction(obj.f_sym, 'Vars',{obj.SymVars.x, obj.SymVars.u, obj.Model.SymParams});
            obj.g_sym_ = matlabFunction(obj.g_sym, 'Vars',{obj.SymVars.x, obj.SymVars.u, obj.Model.SymParams});
        end
        
        function setTables(obj)
            state_desc = ["Battery SOC"; "Motor Current"; "Motor Velocity"; "Shaft Torque"; "Propeller Speed"];
            state_syms = arrayfun(@(x) string(sym2str(x)), obj.SymVars.x);
            obj.StateTable = table(state_syms, state_desc);
            
            in_desc = ["Inverter Input"];
            in_syms = arrayfun(@(x) string(sym2str(x)), obj.SymVars.u);
            obj.InputTable = table(in_syms, in_desc);
            
            out_desc = [state_desc; "Bus Voltage"; "Bus Current"; "Total Thrust"];
            out_syms = arrayfun(@(x) string(sym2str(x)), obj.SymVars.y);
            obj.OutputTable = table(out_syms, out_desc);
        end
        
        
        function linmod = setSymLinearModel(obj)
            u_mod = obj.SymVars.u;
            
            % Cut Battery Dynamics out of Dynamic Model
            x_mod = obj.SymVars.x(2:end);
            f_sym_mod = obj.f_sym(2:end);
            g_sym_mod = obj.g_sym(end);
            
            
            
            A = jacobian(f_sym_mod, x_mod);
            B = jacobian(f_sym_mod, u_mod);
            
            C = jacobian(g_sym_mod, x_mod);
            D = jacobian(g_sym_mod, u_mod);
            
            linmod = LinearModel();
            linmod.A_sym = A;
            linmod.B_sym = B;
            linmod.C_sym = C;
            linmod.D_sym = D;
            
            obj.symLinearModel = linmod;
        end
        
        function [SteadyState, subs_from, subs_to] = calcSteadyState(obj, sym_param_vals)
            arguments
                obj
                sym_param_vals = [];
            end
            
            if ~isempty(obj.Graph.SymParams)
                if ~isempty(sym_param_vals)
                    assert(all(size(sym_param_vals) == size(obj.Graph.SymParams)), 'sym_param_vals argument must be same size as Graph.SymParams');
                else
                    sym_param_vals = obj.Graph.SymParams_Vals;
                end
            end
            
            subs_vars = [sym('x5'); sym('x1'); obj.Graph.SymParams];
            
            %nom_q = obj.Components(1).Nominal_SOC;
            nom_q = 1;
            
            subs_vals = [obj.HoverSpeed; nom_q];
            subs_vals = double(subs(subs_vals, obj.Graph.SymParams, sym_param_vals));
            subs_vals = [subs_vals; sym_param_vals];
            
            dyn_eqns_ss = subs(obj.f_sym(2:end), subs_vars, subs_vals);
            
            solve_vars = [sym('u1'), sym('x2'), sym('x3'), sym('x4')];
            assume(0<sym('u1') & sym('u1')<1);
            digits(8);
            x_ss = vpasolve(dyn_eqns_ss == 0, solve_vars);
            input_sol = min(double(x_ss.u1));
            
            if isempty(input_sol)
                error('No Solution Found');
            end
            
            subs_from = [subs_vars.', solve_vars];
            
            x_ss_array = arrayfun(@(v) min(double(x_ss.(v))), ["u1", "x2", "x3", "x4"]);
            subs_to = double([subs_vals.', x_ss_array]);
            
            SteadyState.x = [subs_vals(2) x_ss_array(2:end) subs_vals(1)]';
            SteadyState.u = input_sol;
            SteadyState.y = double(subs(obj.g_sym, subs_from, subs_to));
        end
        
        function [ssMod, x0, u0, y0] = calcLinearModel(obj, varargin)
            [Xss, subs_from, subs_to] = calcSteadyState(obj, varargin{:});
            
            
            A = subsWrapper(obj.symLinearModel.A_sym);
            B = subsWrapper(obj.symLinearModel.B_sym);
            C = subsWrapper(obj.symLinearModel.C_sym);
            D = subsWrapper(obj.symLinearModel.D_sym);
            
            ssMod = ss(A,B,C,D);
            
            x0 = Xss.x;
            u0 = Xss.u;
            y0 = Xss.y;
            
            function y = subsWrapper(x)
                y = double(subs(x,subs_from,subs_to));
            end
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