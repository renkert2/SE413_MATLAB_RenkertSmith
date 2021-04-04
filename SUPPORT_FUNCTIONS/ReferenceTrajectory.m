classdef ReferenceTrajectory
    %REFERENCETRAJECTORIES Summary of this class goes here
    %   Detailed explanation goes here
    enumeration
        ConstantStep ("ConstantStep")
        Pulses ("Pulses")
    end
        
    properties
        R function_handle
    end
    
    methods
        function obj = ReferenceTrajectory(traj)
            if isa(traj, "string")
                switch traj
                    case "ConstantStep"
                        obj.R = obj.genConstantStep(1);
                    case "Pulses"
                        obj.R = obj.genPulses(29, 2*60, 60, 6, 1);
                end
            end
        end
        
        function plot(obj, t_range, varargin)
           fplot(obj.R,  t_range, varargin{:})
           xlabel('t')
           ylabel('Reference Height')
        end
    end
    
    methods (Static)
        function r = genConstantStep(height)
            r = @(t) (t>0).*height;
        end
        
        function r = genPulses(h,t_high,t_low,N,offset)
            P = t_high + t_low;
            r = @(t) (t>0).*(h*(mod(t,P)<t_high).*(t<=P*N) + offset);
        end
    end
end

