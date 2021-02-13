classdef InverterTypes < uint8
    enumeration
        ConstantLoss (1) % ConstantLoss Type results in logarithmic efficiency curve
        VoltageDependent (2) % Inverter efficiency depends on input voltage
        CurrentDependent (3) % Inverter efficiency depends on current 
    end 
end

