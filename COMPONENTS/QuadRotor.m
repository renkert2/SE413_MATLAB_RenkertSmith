classdef QuadRotor < System
    methods
        function obj = QuadRotor(p)
            arguments
                p.battery Battery = Battery('Name', "Battery");
                p.dcbus DCBus_CurrentEquivalence = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4);
                p.pmsminverter PMSMInverter = PMSMInverter('Name', "Inverter");
                p.pmsmmotor PMSMMotor = PMSMMotor('Name', "Motor");
                p.shaftcoupler ShaftCoupler = ShaftCoupler('Name', "Shaft");
                p.propeller Propeller = Propeller('Name', "Propeller");
            end
            
            replicated_comps = Replicate([p.pmsminverter, p.pmsmmotor, p.shaftcoupler, p.propeller], 4);
            pmsminverters = replicated_comps{1};
            pmsmmotors = replicated_comps{2};
            shaftcouplers = replicated_comps{3};
            propellers = replicated_comps{4};
           
            components = [p.battery, p.dcbus, replicated_comps{:}];
            
            ConnectP = {[p.battery.Ports(1) p.dcbus.Ports(1)]};
            
            for i = 1:4
                ConnectP{end+1,1} = [pmsminverters(i).Ports(1),p.dcbus.Ports(1+i)];
                ConnectP{end+1,1} = [pmsminverters(i).Ports(2), pmsmmotors(i).Ports(1)];
                ConnectP{end+1,1} = [pmsmmotors(i).Ports(2), shaftcouplers(i).Ports(1)];
                ConnectP{end+1,1} = [shaftcouplers(i).Ports(2), propellers(i).Ports(1)];
            end
            
            obj = obj@System(components, ConnectP);
        end
    end
end