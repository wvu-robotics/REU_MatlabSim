classdef Agent < handle   
    properties (Access = public)
        %A 1x2 double detailing the (x,y) position of the agent
        pose;
        
        %A 1x2 double (Vx,Vy) detailing the velocity of the agent is
        %currently traveling in
        velocity;
        
        %A 1x2 double detailing the velocity the agent would travel at if
        %not for collisions
        velocityControl = [0,0];
        
        %A Px2 double where the pose of this agent at time step t was
        %path(t,:)
        path;
        
        %A Px3 double where the color of the agent at time step t was
        %pathColor(t,:). These colors will be shown on the path at the
        %corresponding pose the agent was in at time step t.
        pathColor;
        
        %A 1x2 double describing the position of the goal
        goalPose;
        
        %A 1x3 double describing the current color of the agent. This value
        %will be saved to a new row in pathColor each time step.
        color = [1 1 1];
        
        %An array of external agents that this agent can sense
        measuredAgents = Agent.empty;
        
        %A positive double describing how far away this agent can see. If
        %another agent's pose is less than measureingRange away from this
        %agent's pose, then that other agent is in measuredAgents.
        measuringRange = 3;
        
        %A positive double describing the speed an agent shouldn't exceed
        maxSpeed = 2;
        
        %A positive double detailing the speed an agent wants to move at
        idealSpeed = .5;
    end
    
    properties (Access = private)
        %A positive integer which is the index of this agent in the
        %agentEnv this belongs to
        id;
        
        %A positive double where the agent is physically bounded in a
        %circle centered at pose with this radius
        radius;
        
        %A void function handle that takes in this agent and sets its
        %velocityControl to some value
        controller;
        
        %A positive double which is the time between frames in the sim
        timeStep;
        
        extraProperties = cell.empty;
        extraPropertyList = string.empty;
    end
    
    methods
        %Constructor: Sets the private variables accordingly
        %(see property documentation)
        function obj = Agent(id, radius, timeStep)
            obj.id = id;
            obj.radius = radius;
            obj.timeStep = timeStep;
        end
        
        %Takes the in agentEnv this agent is a part of, and sets
        %measuredAgents accordingly
        function callMeasurement(obj, envObj)
            disp = 0;
            envAgents = envObj.getNumberOfAgents;
            obj.measuredAgents = Agent.empty;
            for i = 1:(envAgents-1)
                if obj.id + i >  envAgents
                    disp = envAgents;
                end
                if norm(envObj.agents(obj.id + i - disp).pose - obj.pose) < obj.measuringRange
                     obj.measuredAgents(end + 1) = envObj.agents(obj.id + i - disp);
                end
            end
        end
        
        %Returns the property 'radius' (see property documentation)
        function radius = getRadius(obj)
            radius = obj.radius;
        end
        
        %Returns a 1x2 double pointing from the agent's pose to its
        %goalPose
        function unitVec = calcIdealUnitVec(obj)
            unitVec = (obj.goalPose - obj.pose)./ ...
                       vecnorm(obj.goalPose - obj.pose, 2, 2);
        end
        
        %Sets the velocityControl according to the controller
        %(see property documentation)
        function callController(obj)
            obj.controller(obj);
        end
        
        %Sets the private property controller (see property documentation)
        function setController(obj,controller)
            obj.controller = controller;
        end 
        
        %Returns the private property timeStep (see property documentation)
        function timeStep = getTimeStep(obj)
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
    end
end

