%% Propeller
prop = Propeller();
prop_model = GraphModel(prop.graph);

%% 
inputs = @(t) [];

disturbances = @(t) [1;1];

t_range = [0,1];

[t,x] = Simulate(sc_model, inputs, disturbances, t_range)


function [t,x] = Simulate(obj, inputs, disturbances, t_range)

[t,x] = ode23t(@(t,x) obj.CalcF(x, inputs(t), disturbances(t)), t_range, [.1;.1;.2]);

plot(t,x)
end