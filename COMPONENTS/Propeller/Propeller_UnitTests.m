%% Propeller
% Test Specimen from "Static Testing of Propulsion Elements for Small Multirotor Unmanned Aerial Vehicles" by Deters et. al.
% DJI Phanton 3
% APC 9.5x5 Propeller

u = symunit;
D = double(separateUnits(unitConvert(9.5*u.in, u.m)));
k_T = 0.12;
k_P = 0.05;

prop = Propeller('Name', 'APC 9.5x5', 'D', D, 'k_T', k_T, 'k_P', k_P);
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

omega_rads = 0:1:1000;
omega_rpm = double(separateUnits(unitConvert(omega_rads*u.rad/u.s, u.rpm)));

thrust = stc.*(omega_rads.^2); % N
thrust_2 = prop.calcThrust(omega_rads);
thrust_lbs = double(separateUnits(unitConvert(thrust*u.N, u.lbf))); % lbf
thrust_lbs_2 = double(separateUnits(unitConvert(thrust_2*u.N, u.lbf))); % lbf

figure
plot(omega_rpm, thrust_lbs, omega_rpm, thrust_lbs_2)
legend(["Thrust 1", "Thrust 2"])