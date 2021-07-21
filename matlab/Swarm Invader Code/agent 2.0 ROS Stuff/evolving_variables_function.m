function [battery_life] = evolving_variables_function(Agent)
% iteration = iteration +1
run('register_variables_for_function.m');
for i = 1:length(Agent.measuredAgents)
    if Agent.measuredAgents(i).getProperty('isEnemy') == false
        fprintf('Battery Life = %.2f\n',Agent.measuredAgents(i).getProperty('Battery_Life'))
        battery_life = battery_life - 1; %This will control number of iterations
        Agent.setProperty('Battery_Life',battery_life)
        distance_from_home = norm(Agent.pose - Agent.goalPose);
        Agent.setProperty('Distance_From_Home',distance_from_home);
        enough_battery_home = distance_from_home / (4*mapSize); % ft / (4 ft / 1%)
        Agent.setProperty('Enough_Battery_Home',enough_battery_home);
    end
    if Agent.measuredAgents(i).getProperty('isEnemy') == true
        distance_from_invader = norm(Agent.pose - Agent.measuredAgents(i).pose)
        Agent.setProperty('Distance_From_Invader',distance_from_invader)
        enough_battery_invader = distance_from_invader / (4*mapSize); % ft / (4 ft / 1%)
        Agent.setProperty('Enough_Battery_Invader',enough_battery_invader)
        enough_battery_home_invader = enough_battery_home + (2*enough_battery_invader) +5;
        Agent.setProperty('Enough_Battery_Home_Invader',enough_battery_home_invader)
    end
end
% Multipled by two to take into account traveling to invader, then heading
% back home after reaching the invader.
% Added a 5% buffer (which is 10ft) to allow them to chase the invader
end

