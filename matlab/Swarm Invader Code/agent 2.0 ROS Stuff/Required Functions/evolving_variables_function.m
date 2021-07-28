function [battery_life] = evolving_variables_function(Agent)
% iteration = iteration +1
run('register_variables_for_function.m'); % allows base workspace
                                          % variables to be used in
                                          % function.
for i = 1:length(Agent.measuredAgents)
    % measuredAgents is what agents are able to see given their measuring
    % range (set in Agent class). each agent is in a different location, if
    % they split up this allows them to cover more ground to detect the
    % invader.
    if Agent.measuredAgents(i).getProperty('isEnemy') == false 
        % Agent ID# 1 has the property 'isEnemy' as true, allows other
        % agents to detect it. 
        x = Agent.measuredAgents(i).getProperty('battery_life');
        fprintf('Battery Life = %.2f\n',Agent.getProperty('battery_life'))
        
        battery_life = Agent.getProperty('battery_life') - .01; %This will control number of iterations
        Agent.setProperty('battery_life',battery_life)
        
        distance_from_home = norm(Agent.pose - Agent.goalPose);
        % goalPose is set to home, which is [0,0]. this can be changed. 
        Agent.setProperty('distance_from_home',distance_from_home);
        
        enough_battery_home = distance_from_home / (4*mapSize); % unit / (4 unit / 1%)
        % robots can move 4 units per 1% battery. this can be changed. 
        Agent.setProperty('enough_battery_home',enough_battery_home);
    
    end
    if Agent.measuredAgents(i).getProperty('isEnemy') == true
        % Agent ID# 1 is considered the enemy and has 'isEnemy' == true
        distance_from_invader = norm(Agent.pose - Agent.measuredAgents(i).pose);
        Agent.setProperty('distance_from_invader',distance_from_invader);
        
        enough_battery_invader = distance_from_invader / (4*mapSize); % ft / (4 ft / 1%)
        Agent.setProperty('enough_battery_invader',enough_battery_invader)
        
        enough_battery_home_invader = enough_battery_home + (2*enough_battery_invader) +5;
        Agent.setProperty('enough_battery_home_invader',enough_battery_home_invader)
    % Multipled by two to take into account traveling to invader, then heading
    % back home after reaching the invader.
    % Added a 5% buffer (which is 10ft) to allow them to chase the invader
    end
end
end

