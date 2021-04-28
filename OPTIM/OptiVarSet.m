classdef OptiVarSet
    %OPTIVARSET Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        OptiVars optiVar
    end
    
    properties (Dependent)
        X0
        LB
        UB
    end
     
    methods
        function obj = OptiVarSet(ov)
            obj.OptiVars = ov;
        end
        
        function i = optiIndex(obj)
            i = vertcat(obj.OptiVars.optiFlag);
        end
        
        function x0 = get.X0(obj)
            x0 = filteredVertCat(obj, 'x0');
        end
        
        function lb = get.LB(obj)
            lb = filteredVertCat(obj, 'lb');
        end
        
        function ub = get.UB(obj)
            ub = filteredVertCat(obj, 'ub');
        end
        
        function setFlag(arg, bool)
            if isa(arg, ["double", "logical"])
                optivars = 
                
    end
    
    methods (Access = private)
        function X = filteredVertCat(obj, prop)
            vec = vertcat(obj.OptiVars.(prop));
            X = vec(optiIndex(obj));
        end
    end
end

