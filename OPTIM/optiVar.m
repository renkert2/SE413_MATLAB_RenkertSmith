classdef optiVar < handle
    %OPTIVAR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Sym string
        
        x0 double
        lb double
        ub double
        
        Description string
        Unit DimVar
        Parent SystemElement
        
        x_opt double
        optiFlag logical = true
    end
    
    methods 
        function obj = optiVar(sym_arg, x0, lb, ub)
            arguments
                sym_arg {mustBeA(sym_arg, ["string", "char"])}
                x0 double
                lb double = -Inf
                ub double = Inf
            end
            
            obj.Sym = string(sym_arg);
            obj.x0 = x0;
            obj.lb = lb;
            obj.ub = ub;
        end
        
        function i = optiIndex(obj)
            i = vertcat(obj.optiFlag);
        end
        
        function i = find(obj, args)
            i = arrayfun(@(x) find(indexBy(obj, 'Sym', x)), string(args));
        end
        
        function o = get(obj, args)
            o = obj(find(obj, args));
        end
        
        function x0 = X0(obj)
            x0 = filteredVertCat(obj, 'x0');
        end
        
        function lb = LB(obj)
            lb = filteredVertCat(obj, 'lb');
        end
        
        function ub = UB(obj)
            ub = filteredVertCat(obj, 'ub');
        end
        
        function xall = XAll(obj, x)
            xall = vertcat(obj.x0);
            xall(optiIndex(obj)) = x;
        end
        
        function setFlag(obj, arg, bool)
            arguments
                obj
                arg
                bool logical
            end
            
            assert(numel(arg) == numel(bool), 'Length of arg and bool args must be the same');
            if isa(arg, "double") || isa(arg, "logical")
                optivars = obj(arg);
            elseif isa(arg,"string")
                optivars = obj(find(obj, arg));
            else
                error('Invalid Argument');
            end
            
            for i = 1:numel(arg)
                optivars(i).optiFlag = bool(i);
            end
        end
        
        function setOptVals(obj, X_opt)
            i = find(optiIndex(obj));
            for j = 1:numel(i)
                obj(j).x_opt = X_opt(j);
            end
        end     
    end
    
    methods (Access = public)
        function X = filteredVertCat(obj, prop)
            vec = vertcat(obj.(prop));
            X = vec(optiIndex(obj));
        end
        
        function i = indexBy(obj, prop, val)
            i = [obj.(prop)] == val;
            if isempty(i)
                error('Val not found')
            end
        end
        
        function obj_array = filterBy(obj, prop, val)
           obj_array = obj(indexBy(obj, prop, val));
        end
    end
end

