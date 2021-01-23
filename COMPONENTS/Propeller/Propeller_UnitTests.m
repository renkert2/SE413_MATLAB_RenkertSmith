%% Propeller
prop = Propeller('Name', "Propeller 1");
prop_model = GraphModel(prop.graph);

%% 
prop_model.StateNames
prop_model.InputNames
prop_model.DisturbanceNames
%% 
inputs = [];

disturbances = [0.1;0]; % Apply 0.1N*m of torque continuously

t_range = [0,100];

[t,x] = Simulate(prop_model, inputs, disturbances, t_range, 'PlotStates', true, 'PlotDisturbances', false, 'PlotInputs', false);