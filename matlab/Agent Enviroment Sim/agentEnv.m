classdef agentEnv < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
     properties (Access = public)
        agents = Agent.empty;
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
        function obj = agentEnv(numberOfAgents, agentRadius, mapSize)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.numberOfAgents = numberOfAgents;
            obj.agentRadius = agentRadius;
            obj.mapSize = mapSize;
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

        function setAgentPositions(obj,pose)
            for i = 1:obj.numberOfAgents
                obj.agents(i).pose = pose(i,:);
                if isempty(obj.agents(i).path)
                    obj.agents(i).path(1,:) = pose(i,:);
                    obj.agents(i).pathColor(1,:) = obj.agents(i).color;
                else
                    obj.agents(i).path(length(obj.agents(i).path(:,1))+1,:) = pose(i,:);
                    obj.agents(i).pathColor(length(obj.agents(i).pathColor(:,1))+1,:) = obj.agents(i).color;
                end
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
        
        function updateGraph(obj)
            set(obj.lineGoalLocations, 'xdata', obj.goalLocations(:,1), ...
                                       'ydata', obj.goalLocations(:,2));
            for i = 1:obj.numberOfAgents
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
    end
end

