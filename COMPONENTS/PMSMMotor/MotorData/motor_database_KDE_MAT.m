fileName = 'motor_database_kde.json';
raw = fileread(fileName);
motor_data = jsondecode(raw);
%%
specs_fields = ["Weight", "Diameter", "Length", "Maximum_Amps"];
const_fields = ["kV", "Rm", "Io"];

for i = 1:numel(motor_data)
    motor = motor_data(i);
    specs = motor.SPECS;
    
    motor_new = struct();
    
    motor_new.MAKE = motor.MAKE;
    motor_new.MODEL = motor.MODEL;
    motor_new.PRICE = motor.PRICE;
    
    motor_new.SPECS.Weight = specs.Motor_Weight.Value / 1000;
    motor_new.SPECS.Diameter = specs.Motor_Diameter.Value / 1000;
    motor_new.SPECS.Length = specs.Motor_Length.Value / 1000;
    try
        motor_new.SPECS.Maximum_Amps = specs.Maximum_Continuous_Current.Value;
    catch
        motor_new.SPECS.Maximum_Amps = NaN;
    end
    
    motor_new.CONSTANTS.kV = specs.Kv.Value; % rpm/V
    motor_new.CONSTANTS.Rm = specs.Rm.Value; % Ohms
    motor_new.CONSTANTS.Io = specs.Io.Value;
    
    motor_data_kde(i) = motor_new;
end

save('motor_data_kde.mat','motor_data_kde');

