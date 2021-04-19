classdef optiVar < compParam
    %OPTIVAR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x0 double
        lb double
        ub double
        
        Enabled logical = true
        scaleFactor double
        Scaled logical = true
    end
    
    methods 
        function obj = optiVar(sym_arg, x0, lb, ub, opts)
            arguments
                sym_arg {mustBeA(sym_arg, ["string", "char"])}
                x0 double
                lb double = -Inf
                ub double = Inf
                opts.Enabled = true
                opts.Scaled = true
                opts.ScaleFactor double = []
                
                opts.Tunable logical = true
                opts.AutoRename logical = false
                opts.Description = ""
                opts.Unit = ""
            end
            
            obj = obj@compParam(sym_arg, x0);
            obj.Tunable = opts.Tunable;
            obj.AutoRename = opts.AutoRename;
            obj.Description = opts.Description;
            obj.Unit = opts.Unit;

            obj.x0 = x0;
            obj.lb = lb;
            obj.ub = ub;
            
            obj.Enabled = opts.Enabled;
            
            if isempty(opts.ScaleFactor)
                obj.scaleFactor = x0;
            else
                obj.scaleFactor = opts.ScaleFactor;
            end
            
        end
        
        function i = isEnabled(obj)
            i = vertcat(obj.Enabled);
        end
        
        
        function x0 = X0(obj)
            x0 = scale(obj, filterEnabled(obj, 'x0'));
            
        end
        
        function lb = LB(obj)
            lb = scale(obj, filterEnabled(obj, 'lb'));
        end
        
        function ub = UB(obj)
            ub = scale(obj, filterEnabled(obj, 'ub'));
        end
        
        function xall = XAll(obj, x)
            xall = vertcat(obj.x0);
            xall(isEnabled(obj)) = unscale(obj,x);
        end
        
        function xall = XAllOpt(obj)
            xall = vertcat(obj.Value);
        end
        
        function setEnabled(obj, arg, bool)
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
                optivars(i).Enabled = bool(i);
            end
        end
        
        function setOptVals(obj, val)
            val = unscale(obj, val);
            i = find(isEnabled(obj));
            for j = 1:numel(i)
                obj(j).Value = val(j);
            end
        end     
    end
    
    methods (Hidden)
        function X = filterEnabled(obj, prop)
            X = filteredProps(obj, prop, isEnabled(obj));
        end
        
        function s = scaleFactors(obj)
            s = filterEnabled(obj, 'scaleFactor');
            i = vertcat(obj.Scaled);
            s(~i) = 1;
        end
        
        function x_s = scale(obj, x)
            x_s = x./scaleFactors(obj);
        end
        
        function x_u = unscale(obj,x)
            x_u = x.*scaleFactors(obj);
        end
    end
end

