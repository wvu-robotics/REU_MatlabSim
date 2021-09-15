% iteration = iteration +1 , can add this
% Script is to constantly evolve variables for use in the state machine
for i = 1:numberOfAgents
fprintf('Battery Life = %.2f\n',battery_life)
battery_life = battery_life - .001; %This will control number of iterations
distance_from_home = round(rand(1)*(100-0)+0); %random number between [0,100]
distance_from_invader = round(rand(1)*(100-0)+0); %random number between [0,100]
enough_battery_home = distance_from_home / (4*mapSize); % ft / (4 ft / 1%)
enough_battery_invader = distance_from_invader / (4*mapSize); % ft / (4 ft / 1%)
enough_battery_home_invader = enough_battery_home + (2*enough_battery_invader) +5;
end
% Multipled by two to take into account traveling to invader, then heading
% back home after reaching the invader.
% Added a 5% buffer (which is 10ft) to allow them to chase the invader

% Any questions can be directed to danielvillarrealusa@gmail.com.