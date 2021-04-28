inv = PMSMInverter('Name', "PMSM Inverter");

%%
inv.Graph.Edges(4).Input = inv.Graph.Inputs(1);
inv.Graph.Edges(4).PowerFlow(2) = Type_PowerFlow("u1*xt");
%%

inv_model = GraphModel(inv.Graph);

%% 

inputs = @(t) heaviside(t-0.05); % Increase output voltage by factor of 2
disturbances = @(t) [heaviside(t-0.05)*10;ones(size(t)).*11.1;zeros(size(t))];

%%
close all
inv_model.Simulate(inputs, disturbances, 0:0.001:0.1);