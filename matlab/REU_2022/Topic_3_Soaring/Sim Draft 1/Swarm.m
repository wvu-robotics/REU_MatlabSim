% Swarm class
classdef Swarm < handle
    properties
        agents
        simLaw
        funcHandle_agentControl
        funcHandle_findNeighborhood
        
        thermalMap
        
        lineCircle = NaN
        lineNeighbors = NaN
        lineRange = NaN
    end
    
    methods
        % Generation Function
        function obj = Swarm(simLaw,thermalMap)
            % Save parameters and agentControlFunc
            obj.simLaw = simLaw;
            obj.thermalMap = thermalMap;
            SL = obj.simLaw;
            
            obj.funcHandle_agentControl = str2func(SL.funcName_agentControl);
            obj.funcHandle_findNeighborhood = str2func(SL.funcName_findNeighborhood);
            
            %% Generate agents
            numAgents = SL.numAgents;   %Get total number of agents
            obj.agents = Agent.empty(0,SL.numAgents);
            obj.agents(1,numAgents) = Agent();  %Fill agents with default constructors of Agent
            
            %Iterate through all agents
            posRange = SL.agentSpawnPosRange;
            velRange = SL.agentSpawnVelRange;
            altiRange = SL.agentSpawnAltiRange;
            for i=1:numAgents
                obj.agents(i).simLaw = SL;
                %Set agent initial position, heading, bank angle, velocity
                obj.agents(i).position(1) = Utility.randIR(posRange(1,1),posRange(2,1));
                obj.agents(i).position(2) = Utility.randIR(posRange(1,2),posRange(2,2));
                obj.agents(i).position(3) = Utility.randIR(altiRange(1),altiRange(2));
                obj.agents(i).heading = Utility.randIR(0,2*pi); %rad
                obj.agents(i).bankAngle = 0; %rad
                obj.agents(i).velocity(1) = Utility.randIR(velRange(1,1),velRange(2,1));
                obj.agents(i).velocity(2) = Utility.randIR(velRange(1,2),velRange(2,2));
            end
        end
        
        % Save Function
        function obj = saveAgentData(obj)
            SL = obj.simLaw;
            numAgents = SL.numAgents;
            for i=1:numAgents
                obj.agents(i).saveData();
            end
        end
        
        % Step Function
        function obj = stepSimulation(obj)
            SL = obj.simLaw;
            numAgents = SL.numAgents;   %Get total number of agents
            for i=1:numAgents
                if obj.agents(i).isAlive
                    currentAgent = obj.agents(i);
                    
                    %Find localAgents
                    localAgents = obj.funcHandle_findNeighborhood(obj,i,SL);
                    
                    %Find thermal strength from ThermalMap
                    thermalStrength = obj.thermalMap.getStrength(currentAgent.position);
                    
                    %Update currentAgent
                    obj.funcHandle_agentControl(currentAgent,localAgents,thermalStrength,[0,0,0], SL);
                end
            end
        end
        
        % Render
        function obj = renderAgents(obj)
            SL = obj.simLaw;
            shownNeighbors = false;
            for i=1:SL.numAgents
                if(~shownNeighbors && (SL.showFixedRadius || SL.showNeighbors || SL.showRange) && obj.agents(i).isAlive)
                    shownNeighbors = true;
                    currentAgent = obj.agents(i);
                    localAgents = obj.funcHandle_findNeighborhood(obj,i,SL);

                    if(SL.showFixedRadius)
                        theta = linspace(currentAgent.heading-SL.fov/2,currentAgent.heading+SL.fov/2,20);
                        xCircle = SL.neighborRadius * cos(theta) + currentAgent.position(1);
                        yCircle = SL.neighborRadius * sin(theta) + currentAgent.position(2);

                        if(class(obj.lineCircle) == "double")
                            obj.lineCircle = line();
                        end
                        obj.lineCircle.XData = xCircle;
                        obj.lineCircle.YData = yCircle;
                        if ~currentAgent.isAlive
                            obj.lineCircle.Visible = 'off';
                        end
                    end

                    % this was commented?
                    if(SL.showNeighbors)
                        numLocalAgents = size(localAgents,2);
                        linePoints = zeros(2,2*numLocalAgents+1);
                        linePoints(1,1) = currentAgent.position(1);
                        linePoints(2,1) = currentAgent.position(2);
                        for j=1:numLocalAgents
                            linePoints(1,2*j) = localAgents(j).position(1);
                            linePoints(2,2*j) = localAgents(j).position(2);
                            linePoints(1,2*j+1) = currentAgent.position(1);
                            linePoints(2,2*j+1) = currentAgent.position(2);
                        end

                        if(class(obj.lineNeighbors) == "double")
                            obj.lineNeighbors = line();
                        end
                        obj.lineNeighbors.XData = linePoints(1,:);
                        obj.lineNeighbors.YData = linePoints(2,:);
                    end

                    if(SL.showRange)
                        theta = linspace(0,2*pi,30);
                        xCircleRange = currentAgent.velocity(1)/currentAgent.vsink * currentAgent.position(3) * cos(theta) + currentAgent.position(1);
                        yCircleRange = currentAgent.velocity(1)/currentAgent.vsink * currentAgent.position(3) * sin(theta) + currentAgent.position(2);

                        if(class(obj.lineRange) == "double")
                            obj.lineRange = line();
                        end
                        if currentAgent.position(3) < 200 && currentAgent.isAlive
                            obj.lineRange.XData = xCircleRange;
                            obj.lineRange.YData = yCircleRange;
                            obj.lineRange.Color = [1,0,0];
                            obj.lineRange.Visible = 'on';
                        else
                            obj.lineRange.Visible = 'off';
                        end

                    end
                end
                
                obj.agents(i).render();
            end
        end
    end
end