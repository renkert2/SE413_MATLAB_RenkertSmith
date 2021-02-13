%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components - Add Symbolic Parameter for Propeller Diameter
pmsmmotor = PMSMMotor('Name', "Motor");
shaftcoupler = ShaftCoupler('Name', "Shaft");
propeller = Propeller('Name', "Propeller", 'D', symParam('PropD',0.1270), 'J', symParam('PropJ', 4.4e-05));

%% Connect Components

components = [pmsmmotor, shaftcoupler, propeller];

ConnectP = {[pmsmmotor.Ports(2), shaftcoupler.Ports(1)];
[shaftcoupler.Ports(2), propeller.Ports(1)]};

   
g_sys = Combine(components, ConnectP);
motor_prop_model = GraphModel(g_sys);


motor_prop_model.StateNames
motor_prop_model.DisturbanceNames

disturbances = [11.1;0;0];
%%

[t,x] = Simulate(motor_prop_model, [], disturbances, [4.4e-5;.1270], [0 10], 'Solver', @ode23tb);