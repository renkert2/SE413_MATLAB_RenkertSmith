classdef Boundary
    %BOUNDARY Used to create a 2D boundary around experimental data and evaluate the distance of any 
    % point to the nearest point on the boundary.  Useful for optimization problems in which you 
    % want to constrain the design variables into a polygonal region.
    %TODO
    % - Add smoothing to remove the sharp creases in the Distance function that may give fmincon trouble
    %   Detailed explanation goes here
    
    properties
        Data (:,2) double
        BoundaryPoints (:,2) double
    end
    
    properties (SetAccess = private)
        X_lb
        X_ub
        X_mean
    end
    
    methods
        function obj = Boundary(data)
            obj.Data = data;
            obj.X_lb = min(obj.Data)';
            obj.X_ub = max(obj.Data)';
            obj.X_mean = mean(obj.Data)';
            bound = boundary(obj.Data(:,1), obj.Data(:,2),0);
            obj.BoundaryPoints = obj.Data(bound,:);
        end
        
        function d = distToBoundary(obj,varargin)
            if nargin == 2
                X = varargin{1};
                X1 = X(1,:);
                X2 = X(2,:);
            elseif nargin == 3
                X1 = varargin{1};
                X2 = varargin{2};
            end
            
            d = obj.p_poly_dist(X1,X2,obj.BoundaryPoints(:,1), obj.BoundaryPoints(:,2),true)';
        end
        
        function l = isInBoundary(obj,varargin)
            d = distToBoundary(obj,varargin{:});
            l = (d <= 0);
        end
        
        function plot(obj)
            scatter(obj.Data(:,1), obj.Data(:,2))
            hold on
            plot(obj.BoundaryPoints(:,1), obj.BoundaryPoints(:,2))
            hold off
            
            title("Boundary")
            xlabel('X_1')
            ylabel('X_2')
        end
    end
    
    methods (Static)
        [d_min, varargout] = p_poly_dist(xp, yp, xv, yv, varargin);
    end
end

