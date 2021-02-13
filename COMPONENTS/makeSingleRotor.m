function [single_rotor_model, components] = makeSingleRotor(opts)
%MAKESINGLEROTOR Summary of this function goes here
%   Detailed explanation goes here
arguments
    opts.battery Battery = Battery('Name', "Battery");
    opts.dcbus DCBus_CurrentEquivalence = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',1);
    opts.pmsminverter PMSMInverter = PMSMInverter('Name', "Inverter");
    opts.pmsmmotor PMSMMotor = PMSMMotor('Name', "Motor");
    opts.shaftcoupler ShaftCoupler = ShaftCoupler('Name', "Shaft");
    opts.propeller Propeller = Propeller('Name', "Propeller");
end

battery = opts.battery;
dcbus = opts.dcbus;
pmsminverter = opts.pmsminverter;
pmsmmotor = opts.pmsmmotor;
shaftcoupler = opts.shaftcoupler;
propeller = opts.propeller;

components = [battery, dcbus, pmsminverter, pmsmmotor, shaftcoupler, propeller];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)]};

ConnectP{end+1,1} = [pmsminverter.Ports(1), dcbus.Ports(2)];
ConnectP{end+1,1} = [pmsminverter.Ports(2), pmsmmotor.Ports(1)];
ConnectP{end+1,1} = [pmsmmotor.Ports(2), shaftcoupler.Ports(1)];
ConnectP{end+1,1} = [shaftcoupler.Ports(2), propeller.Ports(1)];

g_sys = Combine(components, ConnectP);
single_rotor_model = GraphModel(g_sys);

end

