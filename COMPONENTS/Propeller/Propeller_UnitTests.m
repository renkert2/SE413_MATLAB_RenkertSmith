%% Propeller
prop = Propeller('Name', "Propeller 1");
prop_model = GraphModel(prop);

%% 
prop_model.StateNames
prop_model.InputNames
prop_model.DisturbanceNames
%% 
inputs = [];

disturbances = [0.00255*24;0]; % Apply 0.1N*m of torque continuously

t_range = [0,10];

figure
[t,x] = Simulate(prop_model, inputs, disturbances, [], t_range, 'PlotStates', true, 'PlotDisturbances', false, 'PlotInputs', false);

%%  Thrust Curve
stc = prop.square_thrust_coeff; % rad/s
stc2 = 0.05*prop.rho*prop.D^4; % rev/s

omega_rads = 0:1:2500;
omega_revs = omega_rads./(2*pi);

thrust = stc.*(omega_rads.^2); % N
thrust_2 = stc2.*(omega_revs.^2); % N

figure
plot(omega_rads, thrust, omega_rads, thrust_2)
legend(["Thrust 1", "Thrust 2"])