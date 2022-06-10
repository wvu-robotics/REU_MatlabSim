% Swarm class
classdef Swarm
    properties
        agents
        thermalMap
    end
    
    methods
        function obj = Swarm()
            %% Generate agents
            numAgents = SimLaw.numAgents;   %Get total number of agents
            obj.agents(1:numAgents) = Agent();  %Fill agents with default constructors of Agent
            
            %Iterate through all agents
            posRange = SimLaw.agentSpawnPosRange;
            velRange = SimLaw.agentSpawnVelRange;
            for i=1:numAgents
                %Set agent initial position, heading, bank angle, velocity
                obj.agents(i).position(1) = Utility.randIR(posRange(1,1),posRange(2,1));
                obj.agents(i).position(2) = Utility.randIR(posRange(2,1),posRange(2,2));
                obj.agents(i).heading = Utility.randIR(0,2*pi); %rad
                obj.agents(i).bankAngle = 0; %rad
                obj.agents(i).velocity(1) = Utility.randIR(velRange(1,1),velRange(2,1));
                obj.agents(i).velocity(2) = Utility.randIR(velRange(1,2),velRange(2,2));
            end
            
            %% Generate thermal map
            obj.thermalMap = ThermalMap();
        end
        
        function obj = stepSimulation(obj)
            numAgents = SimLaw.numAgents;   %Get total number of agents
            for i=1:numAgents
                currentAgent = obj.agents(i);
                
                %Find local agents for currentAgent
                numLocalAgents = 0;
                localAgentIndices = -1 * ones(1:numAgents);
                for j=1:numAgents
                    if(j==i)
                        continue
                    end
                    otherAgent = obj.agents(j);
                    dist = Utility.isNear(currentAgent,otherAgent,SimLaw.neighborRadius);
                    if(~isnan(dist))
                        numLocalAgents = numLocalAgents + 1;
                        localAgentIndices(numLocalAgents) = j;
                    end
                end
                localAgents(1:numLocalAgents) = obj.agents(localAgentIndices(1:numLocalAgents));
                
                %Find thermal strength from ThermalMap
                %thermalStrength = thermalMap.getStrength(currentAgent.position);
                thermalStrength = 5;
                
                %Update currentAgent
                currentAgent.update(localAgents,thermalStrength);
            end
        end
    end
end