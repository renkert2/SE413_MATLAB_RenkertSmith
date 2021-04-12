classdef MotorProp < System
    methods
        function obj = MotorProp(opts)
            arguments
                opts.Name = "MotorProp"
                opts.pmsmmotor = PMSMMotor('Name', "Motor");
                opts.propeller = Propeller('Name', "Propeller");
            end
            
            comps = [opts.pmsmmotor, opts.propeller];
            obj = obj@System(comps, {});
            obj.Name = opts.Name;
        end
    end
    
    methods
        function createSysGraph(obj)
            Motor = obj.Components(1);
            Prop = obj.Components(2);
            
            V = Motor.Graph.Vertices;
            E = Motor.Graph.Edges;
            
            % Modify Vertices
            V(2).Coefficient = (Motor.J + Prop.J);
            V(2).Parent = obj;
            V(4).Description = "Drag Sink";
            V(4).VertexType = 'Abstract';
            V(4).Parent = obj;
            
            % Modify Edges
            PF = Type_PowerFlow('xt^3');
            E(3).PowerFlow = PF;
            E(3).Coefficient = Prop.square_drag_coeff;
            E(3).Parent = obj;

            
            g = Graph(V, E);
            obj.Graph = g;
            obj.Graph.Parent = obj;
            
            % Ouptuts
            syms omega
            Thrust = Prop.calcThrust(omega);
            Thrust_Fun = symfun(Thrust, omega);
            O(1) = GraphOutput('Description', "Thrust", 'Function', Thrust_Fun, 'Breakpoints', {V(2)});
            O(1).Parent = obj;
            
            Torque = Prop.calcTorque(omega);
            Torque_Fun = symfun(Torque, omega);
            O(2) = GraphOutput('Description', "Torque", 'Function', Torque_Fun, 'Breakpoints', {V(2)});
            O(2).Parent = obj;
            
            obj.Graph.Outputs = O;
            
            % Ports            
            p = Motor.Ports([1,3]);
            obj.Ports = p;
            
            % Extrinsic Props
            obj.extrinsicProps = Combine(vertcat(Motor.extrinsicProps, Prop.extrinsicProps));
            
            % Sym Params
            sym_params = join(vertcat(Motor.SymParams, Prop.SymParams));
            obj.SymParams = sym_params;
            obj.Graph.SymParams = sym_params;
        end
    end
end

