classdef Agent < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
       pose = [0 0];
       velocity = [0 0];
       velocityControl = [0 0];
       path; 
       pathColor;
       previousHeading;
       goalPose = [0 0];
       color = [0 0 0];
       measuredAgents = Agent.empty;
       measuredObstacle = staticObstacle.empty;
       measuringRange = 1000;
       maxSpeed = .5;
       idealSpeed = 1;
       heading = 0;
       broadCollisionSpace;
       needsUpdate = false;
    end
    
    properties (Access = private)  
        shapeID = 0;
      	id;
        shape = circle(.5);
        radius = 1;
        controller;
        timeStep; 
        extraProperties = cell.empty;
        extraPropertyList = string.empty;
    end
    
    methods
        function callMeasurement(obj, envObj) 
            disp = 0;
            obj.measuredAgents = Agent.empty;
            obj.measuredObstacle = staticObstacle.empty;
            for type = [0,1]
                if type == 0
                    numObs= envObj.getNumberOfAgents;
                else
                    numObs = length(envObj.obstacles);
                end
                if any(numObs)
                    for i = 1:(numObs +type - 1)
                        if type == 0
                            if obj.id + i >  numObs
                                disp = numObs;
                            end
                            if norm(envObj.agents(obj.id+i - disp).pose - obj.pose) < obj.measuringRange
                                obj.measuredAgents(end + 1) = envObj.agents(obj.id+i - disp);  
                            end
                        else
                            if norm(envObj.obstacles(i).pose - obj.pose) < obj.measuringRange
                                obj.measuredObstacle(end + 1) = envObj.obstacles(i);  
                            end
                        end
                    end 
                end
            end
        end
        
        function obj = Agent(id, timeStep)
            obj.id = id;
            obj.setShape(obj.shape);
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
        
        function setShape(obj,shape)
            obj.shape = shape;
            X = [min(shape(:,1)) max(shape(:,1))];
            Y = [min(shape(:,2)) max(shape(:,2))];
            obj.broadCollisionSpace = [X(1),Y(1);
                                       X(1),Y(2);
                                       X(2),Y(2);
                                       X(2),Y(1)];
            obj.needsUpdate = true;
        end 
        
        function property = getProperty(obj,propName)
           index = find(contains(obj.extraPropertyList,propName)); 
           property = cell2mat(obj.extraProperties(index));
        end
        
        function setShapeID(obj, id)
            obj.shapeID = id;
        end
        
        function id = getShapeID(obj)
             id = obj.shapeID;
             
        end
        
        function setProperty(obj,propName,input)
           index = find(contains(obj.extraPropertyList,propName)); 
           obj.extraProperties(index) = {[input]};
        end
        
        function shape = getShape(obj)
            shape = obj.shape;
        end
        
        function id = getID(obj)
            id = obj.id;
        end
    end
end

