function [initPositions, goalLocations] = SpawnAgents(numberOfAgents, spawnType, mapSize)

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
        goalLocations = [];
        for i = 1:numberOfAgents
            found = false;
            while found == false
                goalLocations(end + 1, :) = randi([-mapSize,mapSize],1,2);
                for j = 1:size(goalLocations,1) - 1
                    found = true;
                    if goalLocations(j,:) == goalLocations(end,:)
                        found = false;
                        goalLocations(end, :) = [];
                        break;
                    end
                end
            end
        end
        
    case 'opposingGroups'
        initPositions = [];
        for i = 1:numberOfAgents
            found = false;
            if mod(i,2) == 1
                bottomPosBound = round(mapSize*2/3);
                upperPosBound = round(mapSize);
                bottomGoalBound = round(-mapSize);
                upperGoalBound = round(-mapSize*2/3);
            else              
                bottomPosBound = round(-mapSize);
                upperPosBound = round(-mapSize*2/3);
                bottomGoalBound = round(mapSize*2/3);
                upperGoalBound = round(mapSize);
            end
            while found == false
                initPositions(end + 1, :) = randi([bottomPosBound,upperPosBound],1,2);
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
        goalLocations = [];
        for i = 1:numberOfAgents
            if mod(i,2) == 1
                bottomPosBound = round(mapSize*2/3);
                upperPosBound = round(mapSize);
                bottomGoalBound = round(-mapSize);
                upperGoalBound = round(-mapSize*2/3);
            else              
                bottomPosBound = round(-mapSize);
                upperPosBound = round(-mapSize*2/3);
                bottomGoalBound = round(mapSize*2/3);
                upperGoalBound = round(mapSize);
            end
            found = false;
            while found == false
                goalLocations(end + 1, :) = randi([bottomGoalBound,upperGoalBound],1,2);
                for j = 1:size(goalLocations,1) - 1
                    found = true;
                    if goalLocations(j,:) == goalLocations(end,:)
                        found = false;
                        goalLocations(end, :) = [];
                        break;
                    end
                end
            end
        end
end