% Swarm class
classdef Swarm < handle
    properties
        agents
        thermalMap
        simLaw
        agentControlFunc
    end
    
    methods
        % Generation Function
        function obj = Swarm(simLaw)
            % Save parameters and agentControlFunc
            obj.simLaw = simLaw;
            SL = obj.simLaw;
            
            obj.agentControlFunc = str2func(SL.agentControlFuncName);
            
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
            
            %% Generate thermal map
            obj.thermalMap = ThermalMap();
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
                    
                    %Find local agents for currentAgent
                    numLocalAgents = 0;
                    localAgentIndices = -1 * ones(1,numAgents);
                    for j=1:numAgents
                        if(j==i) || (~obj.agents(j).isAlive)
                            continue
                        end
                        otherAgent = obj.agents(j);
                        dist = norm(currentAgent.position - otherAgent.position);
                        if i == 1
                            %fprintf("Agent %g (%g,%g,%g) to %g (%g,%g,%g), dist: %g\n",i,currentAgent.position(1),currentAgent.position(2),currentAgent.position(3),j,otherAgent.position(1),otherAgent.position(2),otherAgent.position(3),dist);
                        end
                        %fprintf("Agent %g Angle: %g\n", i, currentAgent.bankAngle/2/pi*180);
                        if(dist < SL.neighborRadius)
                            numLocalAgents = numLocalAgents + 1;
                            localAgentIndices(numLocalAgents) = j;
                        end
                    end
                    clear localAgents
                    localAgents(1:numLocalAgents) = obj.agents(localAgentIndices(1:numLocalAgents));
                    
                    %Find thermal strength from ThermalMap
                    %thermalStrength = thermalMap.getStrength(currentAgent.position);
                    thermalStrength = SL.tempThermalStrength;
                    
                    %Update currentAgent
                    obj.agentControlFunc(currentAgent,localAgents,thermalStrength,[0,0,0], SL);
                end
            end
        end
        
        % Render
        function obj = renderAgents(obj)
            SL = obj.simLaw;
            for i=1:SL.numAgents
                obj.agents(i).render();
            end
        end
    end
end