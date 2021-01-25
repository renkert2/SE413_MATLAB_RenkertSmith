inv = PMSMInverter('Name', "PMSM Inverter");
inv_model = GraphModel(inv.graph);
%% 
inv_model.StateNames
inv_model.InputNames
inv_model.DisturbanceNames

inputs = [2]; % Increase output voltage by factor of 2
disturbances = [10;11.1;0];


%%
close all
inv_model.Simulate(inputs, disturbances, [0 0.1]);