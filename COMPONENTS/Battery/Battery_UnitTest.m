batt = Battery('Name', "Battery", 'N_p', 2);
batt_model = GraphModel(batt.graph);
%% 
batt_model.StateNames
batt_model.InputNames
batt_model.DisturbanceNames

disturbances = [1;0];


%%
close all
batt_model.Simulate([], disturbances, [0 100]);