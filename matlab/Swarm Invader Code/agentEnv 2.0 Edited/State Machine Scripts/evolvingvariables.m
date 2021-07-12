% iteration = iteration +1
fprintf('Battery Life = %.2f\n',battery_life)
battery_life = battery_life - 1; %This will control number of iterations
% Robot will be losing 1% doing any task, whether they are 
% moving or idle. They can go 2ft for every 1%
initial_number = rand(1)*(1-0)+0; %random number between [0,1]
if initial_number >= 0.5
    initial_number = 1;
    fprintf('The invader has been detected.\n')
else
    initial_number = 0;
    fprintf('No invader has been detected.\n')
end
invader_detected = initial_number;
distance_from_home = round(rand(1)*(100-0)+0); %random number between [0,100]
distance_from_invader = round(rand(1)*(100-0)+0); %random number between [0,100]
enough_battery_home = distance_from_home / (4*mapSize); % ft / (4 ft / 1%)
enough_battery_invader = distance_from_invader / (4*mapSize); % ft / (4 ft / 1%)
enough_battery_home_invader = enough_battery_home + (2*enough_battery_invader) +5;
% Multipled by two to take into account traveling to invader, then heading
% back home after reaching the invader.
% Added a 5% buffer (which is 10ft) to allow them to chase the invader