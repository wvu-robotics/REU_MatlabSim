function [initPositions, goalLocations] = HardwareSpawn(spawnType, numberOfAgents)
switch spawnType
    case 'antipodal'
        initPositions = [-1, 0; 1, 0];
        goalLocations = [0.5, 0; -0.5, 0];  
    case 'random'
        initPositions = randi(40, numberOfAgents, 2) - 20;
        goalLocations = randi(40, numberOfAgents, 2) - 20; 
end
end