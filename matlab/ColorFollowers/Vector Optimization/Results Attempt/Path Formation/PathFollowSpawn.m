function [initPositions, goalLocations] = PathFollowSpawn(numberOfAgents, spawnType, mapSize)

switch spawnType
    case 'random'
        initPositions = [];
        for i = 1:numberOfAgents
            found = false;
            while found == false
                initPositions(end + 1, :) = randi([-mapSize,mapSize],1,2);
                for j = 1:size(initPositions,1) - 1
                    found = true;
                    if initPositions(j,:) == initPositions(end,:)
                        found = false;
                        initPositions(end, :) = [];
                        break;
                    end
                end
            end
        end
        goalLocations = zeros(numberOfAgents,2);
end
        
 
end