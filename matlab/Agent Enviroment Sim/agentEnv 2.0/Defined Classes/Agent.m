%% Agent
% A representation of terrestrial robot.

classdef Agent < handle
    properties (Access = public)
        %% Current State
        
        %1x2 double: The position of the agent
        pose = [0 0];
        
        %1x3 double: The current color of the agent
        color = [0 0 0];
        
        %double: The current heading of the agent
        heading = 0;
        
        %1x2 double: The global current velocity of agent
        velocity = [0 0];
        
        %% Prior States/Path
        
        %Px2 double: path(i,:) was the pose of the agent i time steps after
        %the start of the simulation.
        path;
        
        %Px3 double: pathColor(i,:) was the color of the agent i time steps
        %after the start of the simulation.
        %Precondition: size(pathColor,1) == size(path,1)
        pathColor;
        
        %Px1 double: previousHeading(i,:) was the heading of the agent i
        %time steps after the start of the simulation.
        %Precondition: size(previousHeading,1) == size(path,1)
        previousHeading;
        
        %% Commands
        
        %1x2 double: The global velocity the agent wants to move at
        velocityControl = [0 0];
        
        %double: The angular velocity the agent wants to turn at
        angularVelocityControl = 0;
        
        %positive double: maxSpeed >= norm(velocityControl)
        maxSpeed = 2;
        
        %positive double: The speed the agent wants to move at
        idealSpeed = 1;
        
        %1x2 double: The position the agent wants to get to
        goalPose = [0 0];
        
        %% Measuring
        
        %positive double: The radius in which the agent can detect other
        %agents and obstacles
        measuringRange = 1000;
        
        %Nx1 Agent: The neighboring agents whose poses are within a
        %measuringRange radius of this agent
        measuredAgents = Agent.empty;
        
        %Nx1 staticObstable: The obstacles whose poses are within a
        %measuringRange radius of this agent
        measuredObstacle = staticObstacle.empty;
        
        %% Collision
        
        %4x2 double: Holds the corners of a bounding box that encloses the
        %shape of this agent
        broadCollisionSpace;
        
        %DO NOT MODIFY: logical: Whether or not the agentEnv holding this
        %agent needs to respond to a changed broadCollisionSpace
        needsUpdate = false;
        
        %% ROS Bridge
        
        %ros.Publisher: Publishes global velocity and angular velocity
        %commands of type geometry_msgs.Twist
        publisher;
        
        %ros.Subscriber: Receives position and heading updates of type
        %geometry_msgs.TransformStamped
        subscriber;
        
        %geometry_msgs.Twist: The message to be published
        msgPub;
        
        %geometry_msgs.TransformStamped: The message just received
        msgSub;
    end
    
    properties (Access = private)
        
        %unsigned integer: Used by the agentEnv to group this agent with
        %other similarly shaped agents.
        shapeID = 0;
        
        %unsigned integer: Used by the agentEnv to uniquely identify and
        %agent
      	id;
        
        %Vx2 double: A list of the vertices describing the shape of the
        %agent. The vertices' positions are relative to the agent's pose
        %and heading.
        shape = circle(.5);
        
        %positive double: The radius of a bounding circle around the agent.
        %Formally, this means radius = max(vecnorm(shape,2,2)).
        radius = 1;
        
        %funcHandle(Agent): A function that sets the velocityControl
        controller;
        
        %positive double: The timeStep used by the agentEnv
        timeStep;
        
        %
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
        
        %ros
        function setUpPublisher(obj,topic)
            obj.publisher = rospublisher(topic);
            obj.msgPub = rosmessage(obj.publisher);
        end
        
        function setUpSubscriber(obj,topic)
            obj.subscriber = rossubscriber(topic);
            obj.msgSub = rosmessage(obj.subscriber);
        end
        
        end
end