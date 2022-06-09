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
            agents(1:numAgents) = Agent();  %Fill agents with default constructors of Agent
            
            posRange = SimLaw.agentSpawnPosRange;
            velRange = SimLaw.agentSpawnVelRange;
            %Iterate through all agents
            for i=1:numAgents
                agents(i).position(1) = Utility.randIR(posRange
            end
        end
    end
end