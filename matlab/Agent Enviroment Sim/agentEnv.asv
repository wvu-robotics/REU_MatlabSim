classdef agentEnv < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
     properties (Access = public)
        agents = Agent.empty;
        collisions = 0;
     end
    
     properties (Access = private)
        numberOfAgents;
        agentRadius;
        mapSize;
        timeStep; 
        figPS;
        figA;
        lineAgent;
        textAgentNumber;
        linePath;
        lineGoalLocations;
        goalLocations;
     end
    
    methods
%Constructor
        function obj = agentEnv(numberOfAgents, agentRadius, mapSize, timeStep)
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
                obj.agents(i) = Agent(i, agentRadius);
                obj.lineAgent(i) = line('lineWidth', 1);
                obj.textAgentNumber(i) = text;
                set(obj.textAgentNumber(i), 'String', i)
                set(obj.lineAgent(i),'color', 'b')
            end
        end
% Setters
        function setAgentPositions(obj,pose)
            for i = 1:obj.numberOfAgents
                obj.agents(i).pose = pose(i,:);
                obj.updateAgentPath(i,pose);
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
            set(obj.lineAgent(id), 'Color', color);
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
        function collider(obj)            
            for i = 1:obj.numberOfAgents
                obj.agentCollider(i);
            end
        end
        
        function updateAgentColor(obj,id)
            set(obj.lineAgent(id), 'Color', obj.agents(id).color);
        end
        
        function agentCollider(obj, agent)
                disp = 0;
                for j = 1:(obj.numberOfAgents-1)
                    if  agent + j > obj.numberOfAgents
                        disp = obj.numberOfAgents;
                    end 
                    agentDistance =   obj.agents( agent+j-disp).pose - obj.agents(agent).pose;
                    noCollisionDistance = obj.agents( agent).getRadius + obj.agents(agent+j-disp).getRadius;
                    if  norm(agentDistance) < noCollisionDistance
                       obj.collisions = obj.collisions + 1;
                       obj.agents(agent).pose = -noCollisionDistance*(agentDistance/norm(agentDistance))+ obj.agents(agent+j-disp).pose; 
                    end
                end
        end
        
        function pathVisibility(obj, isVisible)
            for i = 1:obj.numberOfAgents
                set(obj.linePath(i),'visible',isVisible)
            end
        end
        
        function updateAgentPath(obj,agent,pose)
                if isempty(obj.agents(agent).path)
                    obj.agents(agent).path(1,:) = pose(agent,:);
                    obj.agents(agent).pathColor(1,:) = obj.agents(agent).color;
                else
                    obj.agents(agent).path(length(obj.agents(agent).path(:,1))+1,:) = pose(agent,:);
                    obj.agents(agent).pathColor(length(obj.agents(agent).pathColor(:,1))+1,:) = obj.agents(agent).color;
                end       
        end
        
        function updateAgentKinematics(obj)
            pose = obj.getAgentPositions;
            for i = 1:obj.numberOfAgents
               obj.agents(i).pose = obj.agents(i).pose + obj.agents(i).velocity*obj.timeStep;
               obj.updateAgentPath(i,pose);
            end
        end
        
        
        function updateGraph(obj)
            set(obj.lineGoalLocations, 'xdata', obj.goalLocations(:,1), ...
                                       'ydata', obj.goalLocations(:,2));
            for i = 1:obj.numberOfAgents
                obj.updateAgentColor(i);
                drawCircle(obj.lineAgent(i),obj.agents(i).pose(1), ...
                                            obj.agents(i).pose(2), ...
                                            obj.agents(i).getRadius);
                set(obj.textAgentNumber(i), "Position", ...
                                            [obj.agents(i).pose(1)  ...
                                             obj.agents(i).pose(2)]);
                                         
                 yPath = obj.agents(i).path(:,2);
                 yPath(end) = NaN;
                set(obj.linePath(i),'xdata',obj.agents(i).path(:,1), ...
                                    'ydata',yPath, ...
                                    'FaceVertexCData',obj.agents(i).pathColor)
            end
        end
        function tick(obj)
            for i = 1:obj.numberOfAgents
                obj.agents(i).callMeasurement(obj);
                obj.agents(i).callController;     
                obj.agentCollider(i);
                obj.updateAgentKinematics;
            end
            obj.collider;
            obj.updateGraph;
            pause(.001)
        end 
    end
end

