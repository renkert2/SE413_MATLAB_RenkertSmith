%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components
pmsmmotor = PMSMMotor('Name', "Motor");
shaftcoupler = ShaftCoupler('Name', "Shaft");
propeller = Propeller('Name', "Propeller");


%% Connect Components

components = [pmsmmotor, shaftcoupler, propeller];

ConnectP = {[pmsmmotor.Ports(2), shaftcoupler.Ports(1)];
[shaftcoupler.Ports(2), propeller.Ports(1)]};

   
g_sys = Combine(components, ConnectP);
motor_prop_model = GraphModel(g_sys);

save('MotorPropSystem.mat','motor_prop_model');