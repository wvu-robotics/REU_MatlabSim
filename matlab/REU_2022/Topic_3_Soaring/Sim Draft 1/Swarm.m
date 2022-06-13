% Swarm class
classdef Swarm < handle
    properties
        agents = Agent.empty(0,SimLaw.numAgents)
        thermalMap
    end
    
    methods
        function obj = Swarm()
            %% Generate agents
            numAgents = SimLaw.numAgents;   %Get total number of agents
            obj.agents(1,numAgents) = Agent();  %Fill agents with default constructors of Agent
            
            %Iterate through all agents
            posRange = SimLaw.agentSpawnPosRange;
            velRange = SimLaw.agentSpawnVelRange;
            altiRange = SimLaw.agentSpawnAltiRange;
            for i=1:numAgents
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
        
        function obj = saveAgentData(obj)
            numAgents = SimLaw.numAgents;
            for i=1:numAgents
                obj.agents(i).saveData();
            end
        end
        
        function obj = stepSimulation(obj)
            numAgents = SimLaw.numAgents;   %Get total number of agents
            for i=1:numAgents
                currentAgent = obj.agents(i);
                
                %Find local agents for currentAgent
                numLocalAgents = 0;
                localAgentIndices = -1 * ones(1,numAgents);
                for j=1:numAgents
                    if(j==i)
                        continue
                    end
                    otherAgent = obj.agents(j);
                    dist = norm(currentAgent.position - otherAgent.position);
                    if i == 1
                        %fprintf("Agent %g (%g,%g,%g) to %g (%g,%g,%g), dist: %g\n",i,currentAgent.position(1),currentAgent.position(2),currentAgent.position(3),j,otherAgent.position(1),otherAgent.position(2),otherAgent.position(3),dist);
                    end
                    %fprintf("Agent %g Angle: %g\n", i, currentAgent.bankAngle/2/pi*180);
                    if(dist < SimLaw.neighborRadius)
                        numLocalAgents = numLocalAgents + 1;
                        localAgentIndices(numLocalAgents) = j;
                    end
                end
                clear localAgents
                localAgents(1:numLocalAgents) = obj.agents(localAgentIndices(1:numLocalAgents));
                if(numLocalAgents > 0)
                    %fprintf("SwarmNear!\n");
                end
               % if(numLocalAgents == 0)
                %    localAgents = NaN;
                %end
                
                %Find thermal strength from ThermalMap
                %thermalStrength = thermalMap.getStrength(currentAgent.position);
                thermalStrength = 5;
                
                %Update currentAgent
                %fprintf("SwarmLocalAgents Size: (%g,%g)\n",size(localAgents,1),size(localAgents,2));
                currentAgent.update(localAgents,thermalStrength,[0,0,0]);
            end
        end
        
        function obj = renderAgents(obj)
            for i=1:SimLaw.numAgents
                obj.agents(i).render();
            end
        end
    end
end