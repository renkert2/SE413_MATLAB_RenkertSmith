classdef MotorProp < System
    methods
        function obj = MotorProp(opts)
            arguments
                opts.Name = "MotorProp"
                opts.pmsmmotor = PMSMMotor('Name', "Motor");
                opts.propeller = Propeller('Name', "Propeller");
            end
            
            comps = [opts.pmsmmotor, opts.propeller];
            obj = obj@System();
            obj.Components = comps;
            obj.Name = opts.Name;
            obj.DefineElement(); % Don't want to modify parameters or children automatically, all defined explicitly below
        end
    end
    
    methods
        function DefineElement(obj)
            Motor = obj.Components(1);
            Prop = obj.Components(2);
            
            V = copy(Motor.Graph.Vertices);
            E = copy(Motor.Graph.Edges);
            E(1).TailVertex = V(3); E(1).HeadVertex = V(1); % Edge TailVertices and HeadVertices must be reassigned
            E(2).TailVertex = V(1); E(2).HeadVertex = V(2);
            E(3).TailVertex = V(2); E(3).HeadVertex = V(4);
            E(4).TailVertex = V(1); E(4).HeadVertex = V(5);
            E(5).TailVertex = V(2); E(5).HeadVertex = V(5);
            
            % Modify Vertices
            V(2).Coefficient = (Motor.J + Prop.J);
            V(2).Parent = obj;
            V(4).Description = "Drag Sink";
            V(4).VertexType = 'Abstract';
            V(4).Parent = obj;
            
            
            
            % Modify Edges
            PF = Type_PowerFlow('xt^3'); % Additional powerflow type for propeller
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
            p(1) = ComponentPort('Description',"Voltage Input",'Element',E(1));
            p(2) = ComponentPort('Description',"Heat Sink",'Element',V(5));
            obj.Ports = p;
            
            % Params
            % I'll wait to combine the extrinsic params until the QuadRotor
            params = unique(vertcat(Motor.Params, Prop.Params));
            obj.Params = params;
        end
    end
end

