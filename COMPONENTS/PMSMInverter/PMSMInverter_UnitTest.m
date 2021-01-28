inv = PMSMInverter('Name', "PMSM Inverter");

%%
inv.graph.Edges(4).Input = inv.graph.Inputs(1);
inv.graph.Edges(4).PowerFlow(2) = Type_PowerFlow("u1*xt");
%%

inv_model = GraphModel(inv.graph);
%% 
inv_model.StateNames
inv_model.InputNames
inv_model.DisturbanceNames

inputs = @(t) heaviside(t-0.05); % Increase output voltage by factor of 2
disturbances = @(t) [heaviside(t-0.05)*10;ones(size(t)).*11.1;zeros(size(t))];


%%
close all
inv_model.Simulate(inputs, disturbances, 0:0.001:0.1);