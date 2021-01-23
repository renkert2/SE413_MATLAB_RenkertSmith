motor = PMSMMotor('Name', "PMSM Motor");
motor_model = GraphModel(motor.graph);
%% 
motor_model.StateNames
motor_model.InputNames
motor_model.DisturbanceNames

input_voltage = 11.1;
load_torque = 0.1;

disturbances = @(t) [input_voltage*ones(size(t)); load_torque*heaviside(t-0.25); 0*ones(size(t))]; % Apply load torque at 0.25 seconds
%%
motor_model.Simulate([], disturbances, [0 1]);
motor_model.Simulate([], disturbances, [0 1], 'PlotStates', false, 'PlotDisturbances', true);