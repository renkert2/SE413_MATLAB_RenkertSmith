classdef InverterTypes < uint8
    enumeration
        ConstantLoss (1) % ConstantLoss Type results in logarithmic efficiency curve
        LinearLoss (2) % LinearLoss type multiplies the constant loss term by input u, creating a constant efficiency curve
        SigmoidalLoss (3) % Work in progress, something like sig(u)*R_2 would allow the efficiency curve to be somewhat logarithmic without power flowing backwards at 0 input.  
    end 
end

