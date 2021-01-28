%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components

battery = Battery('Name', "Battery");
dcbus = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4, 'R', 0);

for i = 1:4 % Each of the following components occur four times in a quadrotor
    pmsminverter(i) = PMSMInverter('Name', sprintf("Inverter %d",i));
    pmsmmotor(i) = PMSMMotor('Name', sprintf("Motor %d", i));
    shaftcoupler(i) = ShaftCoupler('Name', sprintf("Shaft %d",i));
    propeller(i) = Propeller('Name', sprintf("Prop %d",i));
end

%% Connect Components

components = [battery, dcbus, pmsminverter, pmsmmotor, shaftcoupler, propeller];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)]};

for i = 1:4
    ConnectP{end+1,1} = [pmsminverter(i).Ports(1),dcbus.Ports(1+i)];
    ConnectP{end+1,1} = [pmsminverter(i).Ports(2), pmsmmotor(i).Ports(1)];
    ConnectP{end+1,1} = [pmsmmotor(i).Ports(2), shaftcoupler(i).Ports(1)];
    ConnectP{end+1,1} = [shaftcoupler(i).Ports(2), propeller(i).Ports(1)];
end
   
g_sys = Combine(components, ConnectP);
quad_rotor_model = GraphModel(g_sys);

save('QuadRotorSystem','quad_rotor_model');