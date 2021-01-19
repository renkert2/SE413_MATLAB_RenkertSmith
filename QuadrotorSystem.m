%% Init

addpath(genpath('COMPONENTS'));

%% Instantiate Components

battery = Battery();
dcbus = DCBus_CurrentEquivalence('N_inputs',1,'N_outputs',4);

for i = 1:4 % Each of the following components occur four times in a quadrotor
    pmsminverter(i) = PMSMInverter();
    pmsmmotor(i) = PMSMMotor();
    shaftcoupler(i) = ShaftCoupler();
    propeller(i) = Propeller();
end

%% Connect Components

components = [battery, dcbus, pmsminverter, pmsmmotor, shaftcoupler, propeller];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)]};

for i = 1:4
    ConnectP{end+1,1} = [dcbus.Ports(1+i), pmsminverter(i).Ports(1)];
    ConnectP{end+1,1} = [pmsminverter(i).Ports(2), pmsmmotor(i).Ports(1)];
    ConnectP{end+1,1} = [pmsmmotor(i).Ports(2), shaftcoupler(i).Ports(1)];
    ConnectP{end+1,1} = [shaftcoupler(i).Ports(2), propeller(i).Ports(1)];
end
   
g_sys = Combine(components, ConnectP);
%sys_model = GraphModel(g_sys)

%% Plot Graph
g_sys.plot()