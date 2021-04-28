classdef Frame < SystemElement
    %FRAME Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Mass = extrinsicProp("Mass", 0.284 - 0.080 - 4*0.008 - 4*0.04)
    end
    
    methods
        function obj = Frame(varargin)
            obj = obj@SystemElement(varargin{:});
            obj.init_super();
        end
    end
end

