%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components

battery = Battery('Name', "Battery");
dcbus = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',1, 'R', 0);


pmsminverter = PMSMInverter('Name', "Inverter", 'L', 1e-5, 'C', 1e-5);
pmsmmotor = PMSMMotor('Name', "Motor");
shaftcoupler = ShaftCoupler('Name', "Shaft");
propeller = Propeller('Name', "Propeller");


%% Connect Components

components = [battery, dcbus, pmsminverter, pmsmmotor, shaftcoupler, propeller];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)]};

ConnectP{end+1,1} = [pmsminverter.Ports(1), dcbus.Ports(2)];
ConnectP{end+1,1} = [pmsminverter.Ports(2), pmsmmotor.Ports(1)];
ConnectP{end+1,1} = [pmsmmotor.Ports(2), shaftcoupler.Ports(1)];
ConnectP{end+1,1} = [shaftcoupler.Ports(2), propeller.Ports(1)];

   
g_sys = Combine(components, ConnectP);
single_rotor_model = GraphModel(g_sys);

save('SingleRotorSystem.mat','single_rotor_model');