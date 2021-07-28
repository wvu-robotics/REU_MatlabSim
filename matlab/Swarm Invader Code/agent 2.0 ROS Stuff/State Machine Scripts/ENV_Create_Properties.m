% This script is to be used in the main launch file. It is to create
% prviate properties in the launch file for the ENV class, which interacts
% with the Agent class. These properties are accessed by function files to
% run the state machine and other supporting files. The goal is to have
% each individual agent have and calculate its individual variables
% depending on its surroundings and current state. 
for i = 1:numberOfAgents
    if ENV.agents(i).getProperty('isEnemy') == false
        ENV.agents(i).createProperty('battery_life',battery_life);
        ENV.agents(i).createProperty('current_state',current_state);
        ENV.agents(i).createProperty('wallet',wallet);
        ENV.agents(i).createProperty('charging_iterations',charging_iterations);
        ENV.agents(i).createProperty('searching_iterations',searching_iterations);
        ENV.agents(i).createProperty('gathering_iterations',gathering_iterations);
        ENV.agents(i).createProperty('outstanding_distance',outstanding_distance);
        ENV.agents(i).createProperty('invader_detected',invader_detected);
    	ENV.agents(i).createProperty('distance_from_home',distance_from_home);
        ENV.agents(i).createProperty('distance_from_invader',distance_from_invader);
        ENV.agents(i).createProperty('enough_battery_home_invader',enough_battery_home_invader);
    end
end

% Any questions can be directed to danielvillarrealusa@gmail.com.