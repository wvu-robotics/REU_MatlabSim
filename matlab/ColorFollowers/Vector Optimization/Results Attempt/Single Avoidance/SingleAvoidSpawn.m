function [initPositions, goalLocations] = SingleAvoidSpawn(spawnType, numberOfAgents)
switch spawnType
    case 'antipodal'
        initPositions = [-10, 0; 10, 0; 0, 10; 0, -10];
        goalLocations = [10, 0; -10, 0; 0, -10; 0, 10];  
    case 'random'
        initPositions = randi(40, numberOfAgents, 2) - 20;
        goalLocations = randi(40, numberOfAgents, 2) - 20; 
end
end