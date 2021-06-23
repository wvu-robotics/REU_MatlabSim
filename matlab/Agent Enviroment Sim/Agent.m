classdef Agent < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
       pose;
       path 
       pathColor;      
       goalPose;
       color = [1 1 1];
    end
    
    properties (Access = private)  
        id;
        radius;
    end
    
    methods
        function obj = Agent(id, radius)
            obj.id = id;
            obj.radius = radius;
        end
        function radius = getRadius(obj)
            radius = obj.radius;
        end
        function normVec = calcIdealNormVec(obj)
            normVec = (obj.goalPose - obj.pose)./ ...
                       vecnorm(obj.goalPose - obj.pose, 2, 2);
        end
    end
end

