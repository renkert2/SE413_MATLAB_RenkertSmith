bus_alg = DCBus_CurrentEquivalence('Name', "DC Bus Alg", 'N_outputs',4,'R', 1);
bus_model_alg = GraphModel(bus_alg.graph);

bus_dyn = DCBus_CurrentEquivalence('Name', "DC Bus Dyn", 'N_outputs',4, 'L', 1e-3, 'C', 1e-3, 'R', 1);
bus_model_dyn = GraphModel(bus_dyn.graph);
%% 
bus_model_alg.StateNames
bus_model_alg.InputNames
bus_model_alg.DisturbanceNames

disturbances = [10; 10; 10; 10;0];

close all

subplot(1,2,1)
bus_model_alg.Simulate([], disturbances, [0 0.1]);
subplot(1,2,2)
bus_model_dyn.Simulate([], disturbances, [0 0.1]);
plt2 = gcf;