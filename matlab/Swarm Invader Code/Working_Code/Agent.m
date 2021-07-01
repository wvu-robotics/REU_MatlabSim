classdef Agent < handle
    
    properties (Access = public)
       pose;
       velocity;
       velocityControl = [0 0];
       path; 
       pathColor;      
       goalPose;
       color = [0 0 0];
       measuredAgents = Agent.empty;
       measuringRange = 100;
       maxSpeed = .5;
       idealSpeed = 1;
    end
    
    properties (Access = private)  
        id;
        radius;
        controller;
        timeStep; 
        extraProperties = cell.empty;
        extraPropertyList = string.empty;
    end
    
    methods
        function callMeasurement(obj, envObj) 
            disp = 0;
            envAgents = envObj.getNumberOfAgents; 
            obj.measuredAgents = Agent.empty;
            for i = 1:(envAgents-1)
                if obj.id + i >  envAgents
                    disp = envAgents;
                end
                if norm(envObj.agents(obj.id+i - disp).pose - obj.pose) < obj.measuringRange
                     obj.measuredAgents(end + 1) = envObj.agents(obj.id+i - disp);  
                end
            end
        end
        
        function obj = Agent(id, radius, timeStep)
            obj.id = id;
            obj.radius = radius;
            obj.timeStep = timeStep;
        end
        
        function radius = getRadius(obj)
            radius = obj.radius;
        end
        
        function unitVec = calcIdealUnitVec(obj)
            unitVec = (obj.goalPose - obj.pose)./ ...
                       vecnorm(obj.goalPose - obj.pose, 2, 2);
        end
        
        function callController(obj)
            obj.controller(obj);
        end
        
        function setController(obj,controller)
            obj.controller = controller;
        end 
        
        function timeStep= getTimeStep(obj)
            timeStep = obj.timeStep;
        end
        
        function createProperty(obj,propName, input)
           obj.extraPropertyList(length(obj.extraPropertyList) + 1) = propName; 
           obj.extraProperties(length(obj.extraProperties) + 1) = {[input]};
        end
        
        function property = getProperty(obj,propName)
           index = find(contains(obj.extraPropertyList,propName)); 
           property = cell2mat(obj.extraProperties(index));
        end
        
        function setProperty(obj,propName,input)
           index = find(contains(obj.extraPropertyList,propName)); 
           obj.extraProperties(index) = {[input]};
        end
        function id = getID(obj)
            id = obj.id;
        end 
        
    end
end

