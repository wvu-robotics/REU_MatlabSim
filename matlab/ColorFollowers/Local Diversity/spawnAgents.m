%% spawnAgents(environment, numToSpawn, mapSize, spawnType)
%   This spawns agents for the Color_Followers_Local_Diversity script
%   depending on the specified "spawnType"

function environment = spawnAgents(environment, numToSpawn, mapSize, spawnType)

switch spawnType
    
    case 'RandomSpawn'
        %Place agents randomly on the map
        for i = 1:numToSpawn
            new = false;
            while not(new)
                randx = randi(mapSize);
                randy = randi(mapSize);
                if environment.occupancy(randx,randy) == 0
                    new = true;
                end
            end
            environment.occupancy(randx,randy) = 1; %The row coordinate is x, the column coordinate is y
            %Set the goal positions of each agent to a random point on the map
            environment.goalLocation(randx,randy,:) = randi(mapSize,[1,1,2]) .* environment.occupancy(randx,randy);
        end
        
        
    case 'CoordinatedSpawn'
      %Place agents in the top left corner of the map
        for i = 1:numToSpawn
            new = false;
            while not(new)
                randx = randi(round(mapSize/4));
                randy = randi(round(mapSize/4));
                if environment.occupancy(randx,randy) == 0
                    new = true;
                end
            end
            environment.occupancy(randx,randy) = 1; %The row coordinate is x, the column coordinate is y
            
            %Set the goal positions of each agent to a random point on the map
            environment.goalLocation(randx,randy,:) = randi(mapSize/4,[1,1,2]) .* environment.occupancy(randx,randy); 
            environment.goalLocation(randx,randy,:) = environment.goalLocation(randx,randy,:) + mapSize*3/4;
        end
        
        
    case 'OpposingCoordinatedSpawn'
        for i = 1:numToSpawn
            new = false;
            while not(new)
                randx = randi(round(mapSize/4));
                randy = randi(round(mapSize/4));
                if mod(i,2) == 0
                   randy = randy + mapSize*3/4; 
                   environment.goalLocation(randx,randy,:) = randi(round(mapSize/4),[1,1,2]); 
                   environment.goalLocation(randx,randy,1) = environment.goalLocation(randx,randy,1) + mapSize*3/4;
                else
                   environment.goalLocation(randx,randy,:) = randi(round(mapSize/4),[1,1,2]); 
                   environment.goalLocation(randx,randy,:) = environment.goalLocation(randx,randy,:) + mapSize*3/4;
                end
                if environment.occupancy(randx,randy) == 0
                    new = true;
                end
            end
            environment.occupancy(randx,randy) = 1; %The row coordinate is x, the column coordinate is y
        end
end
end