classdef agentEnv1 < handle

     properties (Access = public)
        agents = Agent1.empty;
        realTime = true;
        collisions = 0;
     end
    
     properties (Access = private)
        numberOfAgents;
        agentRadius;
        mapSize;
        timeStep; 
        figPS;
        figA;
        lineAgent = patch;
        textAgentNumber = text;
        linePath = patch;
        lineGoalLocations = line;
        goalLocations;
        isVisible;
     end
    
    methods
%Constructor
        function obj = agentEnv1(numberOfAgents, agentRadius, mapSize, timeStep)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.numberOfAgents = numberOfAgents;
            obj.agentRadius = agentRadius;
            obj.mapSize = mapSize;
            obj.timeStep = timeStep;
            obj.figPS = figure('Name', 'Position Space');
            axis([-mapSize mapSize -mapSize mapSize]);
            obj.lineGoalLocations = line('Marker', '*', ...
                                   'LineStyle', 'none', ...
                                   'Color', [1 0 0]);
            for i = 1:numberOfAgents
                obj.linePath(i) = patch('EdgeColor','interp','MarkerFaceColor','flat');
            end
            for i = 1:numberOfAgents
                obj.agents(i) = Agent1(i, agentRadius,timeStep);
                obj.lineAgent(i) = patch('lineWidth', 1, 'facecolor', 'none');
                obj.textAgentNumber(i) = text;
                set(obj.textAgentNumber(i), 'String', i)
                set(obj.lineAgent(i),'edgecolor', 'b')
            end
        end
% Setters
        function setAgentPositions(obj,pose)
            for i = 1:obj.numberOfAgents
                obj.agents(i).pose = pose(i,:);
                %obj.updateAgentPath(i,pose(i,:));
            end 
        end
        

        function setAgentVelocities(obj, vel)
            for i = 1:obj.numberOfAgents
                obj.agents(i).velocity  = vel(i,:);
            end
        end
        
        function setGoalPositions(obj,goalPose)
            for i = 1:obj.numberOfAgents
                obj.agents(i).goalPose = goalPose(i,:);
            end 
            obj.goalLocations = goalPose;
        end
        
        function setAgentColor(obj,id,color)
            obj.agents(id).color = color;
            set(obj.lineAgent(id), 'edgecolor', color);
        end
        
%getters       
        function pose = getAgentPositions(obj)
            pose = zeros(obj.numberOfAgents, 2); 
            for i = 1:obj.numberOfAgents
                pose(i,:) = obj.agents(i).pose; 
            end
        end
        
        function vel = getAgentVelocities(obj)
            vel = zeros(obj.numberOfAgents, 2); 
            for i = 1:obj.numberOfAgents
                vel(i,:) = obj.agents(i).velocity; 
            end
        end
        
        function numAgents = getNumberOfAgents(obj)
            numAgents = obj.numberOfAgents;
        end 
%functionaility
        function physics(obj, id)
                controllerPose = obj.findAgentControllerKinematics(id);
                hasCollided = obj.agentCollider(id, controllerPose, false);
                if hasCollided
                   obj.collisions = obj.collisions + 1;
                end
                obj.updateAgentKinematics(id);
                obj.updateAgentPath(id,obj.agents(id).pose);
        end
        
        function updateAgentColor(obj,id)
            obj.lineAgent(id).EdgeColor = obj.agents(id).color;
        end
        
        function collision = agentCollider(obj, Agent1, controllerPose, hasCollided)
                collision = hasCollided;
                disp = 0;
                for j = 1:(obj.numberOfAgents-1)
                    if  Agent1 + j > obj.numberOfAgents
                        disp = obj.numberOfAgents;
                    end 
                    agentDistance =   obj.agents( Agent1+j-disp).pose - controllerPose;
                    noCollisionDistance = obj.agents( Agent1).getRadius + obj.agents(Agent1+j-disp).getRadius;
                     x = norm(agentDistance);
                    if ~(abs(x-noCollisionDistance) < .0001)
                        if x < noCollisionDistance
                            hasCollided = true;
                            newPose = -noCollisionDistance*(agentDistance/norm(agentDistance))+ obj.agents(Agent1+j-disp).pose;
                            obj.agents(Agent1).velocity = (newPose - obj.agents(Agent1).pose)/obj.timeStep;
                            collision = obj.agentCollider(Agent1 , newPose, hasCollided);
                            break
                        end
                    end
                end
                if ~hasCollided
                    obj.agents(Agent1).velocity = (controllerPose - obj.agents(Agent1).pose)/obj.timeStep;
                end
        end
        
        function pathVisibility(obj, isVisible)
            for i = 1:obj.numberOfAgents
                set(obj.linePath(i),'visible',isVisible)
            end
            obj.isVisible = isVisible;
        end
        
        function updateAgentPath(obj,Agent1,pose)
                if isempty(obj.agents(Agent1).path)
                    obj.agents(Agent1).path(1,:) = pose;
                    obj.agents(Agent1).pathColor(1,:) = obj.agents(Agent1).color;
                else
                    obj.agents(Agent1).path(length(obj.agents(Agent1).path(:,1))+1,:) = pose;
                    obj.agents(Agent1).pathColor(length(obj.agents(Agent1).pathColor(:,1))+1,:) = obj.agents(Agent1).color;
                end       
        end
        
        function updateAgentKinematics(obj,id)
               obj.agents(id).pose = obj.agents(id).pose + obj.agents(id).velocity*obj.timeStep;
        end
        
        function newPose = findAgentControllerKinematics(obj, id)
               newPose = obj.agents(id).pose + obj.agents(id).velocityControl*obj.timeStep;
        end

        function updateGraph(obj)
            obj.lineGoalLocations.XData =  obj.goalLocations(:,1);
            obj.lineGoalLocations.YData = obj.goalLocations(:,2);
            for i = 1:obj.numberOfAgents
                obj.updateAgentColor(i);
                drawCircle(obj.lineAgent(i),obj.agents(i).pose(1), ...
                                            obj.agents(i).pose(2), ...
                                            obj.agentRadius);
                obj.textAgentNumber(i).Position = [obj.agents(i).pose(1)  ...
                                                   obj.agents(i).pose(2)];     
                                         
                 if obj.isVisible
                    obj.linePath(i).XData = obj.agents(i).path(:,1);
                    obj.linePath(i).YData = [obj.agents(i).path(1:(end-1),2); NaN];
                    obj.linePath(i).FaceVertexCData = obj.agents(i).pathColor;
                 end
            end
        end
        
        
        
        function tick(obj)
           tStart = cputime;
            for i = 1:obj.numberOfAgents
                obj.agents(i).callMeasurement(obj);
                obj.agents(i).callController;     
                obj.physics(i);
                obj.updateGraph;
            end

            tEnd = cputime - tStart;
            if obj.realTime
                if tEnd > obj.timeStep
                    pause(.0001);
                else
                    pause(obj.timeStep - tEnd);
                end
            else
                pause(.0001);
            end
            
        end 
    end
end

