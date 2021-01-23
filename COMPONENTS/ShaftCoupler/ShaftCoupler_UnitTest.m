sc = ShaftCoupler('Name', "ShaftCoupler", 'k', 200e3);
sc_model = GraphModel(sc.graph);
%% 
sc_model.StateNames
sc_model.InputNames
sc_model.DisturbanceNames

in_speed = 1;

disturbances = @(t) [in_speed*ones(size(t)); in_speed*(1-heaviside(t-0.1)+2*heaviside(t-0.2)-1*heaviside(t-0.3))]; % Apply load torque at 0.25 seconds
%%
sc_model.Simulate([], disturbances, [0 1]);
sc_model.Simulate([], disturbances, [0 1], 'PlotStates', false, 'PlotDisturbances', true);