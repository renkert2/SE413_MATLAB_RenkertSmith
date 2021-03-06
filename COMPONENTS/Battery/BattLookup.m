classdef BattLookup < handle
    %BATTLOOKUP Summary of this class goes here
    %   Detailed explanation goes here 
    properties
        SOC (:,1) double
        V_OCV (:,1) double
        V_OCV_nominal double
    end
    
    methods
        function obj = BattLookup(q,v,v_nom)
            obj.SOC = q;
            obj.V_OCV = v;
            obj.V_OCV_nominal = v_nom;
        end
        
        function set.SOC(obj, soc)
            if ~isempty(obj.V_OCV)
                assert(numel(soc) == numel(obj.V_OCV), 'Vectors must be of same length');
            end
            obj.SOC = soc;
        end
        
        function set.V_OCV(obj, vocv)
            if ~isempty(obj.SOC)
                assert(numel(vocv) == numel(obj.SOC), 'Vectors must be of same length');
            end
            obj.V_OCV = vocv;
        end
    end
end

