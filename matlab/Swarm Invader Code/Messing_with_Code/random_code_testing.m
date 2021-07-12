%Code by Daniel Villarreal

% Go to cmd window and set avereage_iterations , counter, and true_average
% to zero. These values will not be cleared when the script is run again
% and will help when trying to calculate the average number of iterations
% possible before the robot's battery is depleted.
average_iterations = 0;
counter = 0;
true_average = 0;
clearvars -except average_iterations counter true_average    %deletes all variables except X in workspace
clc
close all

%% Defined Variables
battery_life = 100;
current_state = 0; % States are listed below:
                        % Idle = 0                      ; w = $1
                        % On Alert = 1                  ; w = $2
                        % In Pursuit = 2                ; w = $5
                        % Deciding = 3                  ; w = $1
                        % Returning Home = 4            ; w = $1
                        % Protecting Home = 5           ; w = $2
                        % Charging = 6                  ; w = $1
                        % Searching = 7                 ; w = $2
                        % Gathering = 8                 ; w = $5
                        % Shut-down (battery = 1%) = 9  ; w = -$2
iteration = 0;
wallet = 0; % This will start them out with $0
charging_iterations = 0;
searching_iterations = 0;
gathering_iterations = 0;
deciding_number = 0;
outstanding_distance = 0;
%% Evolving Variables
if battery_life > 0
    iteration = iteration +1
    fprintf('Battery Life = %.2f\n',battery_life)
    battery_life = battery_life - 1; %This will control number of iterations
    % Robot will be losing 1% doing any task, whether they are moving or
    % idle. They can go 2ft for every 1%

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
enough_battery_home = distance_from_home / 4; % ft / (4 ft / 1%)
enough_battery_invader = distance_from_invader / 4; % ft / (4 ft / 1%)
enough_battery_home_invader = enough_battery_home + (2*enough_battery_invader) +5;
% Multipled by two to take into account traveling to invader, then heading
% back home after reaching the invader.
% Added a 5% buffer (which is 10ft) to allow them to chase the invader

%% Battery Check
if battery_life < enough_battery_home
    current_state = 9;
    fprintf('State: Shutdown\n')
    battery_life = battery_life - enough_battery_home;
    outstanding_distance = abs(4*battery_life); % 4ft/1% * battery life = ft
    battery_life = 0;
    fprintf('Robot died %.2f ft. from home.\n',outstanding_distance)
    wallet = wallet -2;
    return

elseif battery_life == enough_battery_home
    current_state = 4;
    wallet = wallet + 1;
    fprintf('State: Returning Home')
    
end
%% Detect Invader
if invader_detected == 1 % There has been an invader detected if == 1.
    if battery_life >= enough_battery_home_invader 
                          % Checking to see if their battery life is enough
                          % to chase invader and get back home.
        current_state = 1; % If they have enough battery, they are on alert.
        fprintf('State: On Alert\n')
        wallet = wallet + 2;
    else
        current_state = 4; % If their battery is below 50%
                                   % then their current state is 
                                   % returning home.
        wallet = wallet + 1;
        fprintf('State: Returning Home\n')
        battery_life = battery_life - enough_battery_home;
    end
elseif battery_life >= enough_battery_home
    current_state = 0; % If invader_detected == 0 then their
                       % current state is 0. 
        fprintf('State: Idle\n')
        wallet = wallet + 1;
else
    current_state = 4;
    wallet = wallet + 1;
    fprintf('State: Returning Home\n')
    battery_life = battery_life - enough_battery_home;
end

%% Invader Detected
if current_state == 1 % Battery life has been confirmed to be enough to go
                      % to invader and back home. Now they are deciding what to do
                               % to defend against the invader
    if distance_from_home <= 50 % Robots close to home will return home to
                                % protect it.
        difference_invader_home = abs(distance_from_invader - distance_from_home);

            if distance_from_invader <= distance_from_home || difference_invader_home <= 5 
                   current_state = 2;
                   wallet = wallet +5;
                   fprintf('State: In Pursuit\n')
            else
                current_state = 4;
                wallet = wallet + 1;
                fprintf('State: Returning Home\n')
                battery_life = battery_life - enough_battery_home;
            end
    else
        current_state = 2; % If distance_from_home > 50, then they will
                           % pursue the invader
        wallet = wallet + 5;
        fprintf('State: In Pursuit\n')
    end
end
%% No Invader Detected
if current_state == 0 % If there is no invader detected then they
                           % begin mapping their choices while idle.
    if battery_life >= (enough_battery_home +5) % If battery life is enough to
                                              % get home then they plan their
                                              % next move
        current_state = 3;
        wallet = wallet +1;
        fprintf('State: Deciding\n')
        deciding_number = rand(1)*(1-0)+0; %random number between [0,1]

            if deciding_number >= 0.5
                 current_state = 7;
                 wallet = wallet + 2;
                 dit = 0; % so I can do a for loop

                 for dit = dit:10
                     if battery_life >= (enough_battery_home +1)
                         battery_life = battery_life - 1;
                         fprintf('Status: Searching\n')
                         searching_iterations = searching_iterations +1;
                         foraging_number = rand(1)*(1-0)+0;

                            if foraging_number >= 0.95 && battery_life >= enough_battery_home
                                    current_state = 8;
                                    fprintf('State: Gathering\n')
                                    wallet = wallet + 5;
                                    battery_life = battery_life - (2*enough_battery_home);
                                    gathering_iterations = gathering_iterations + 1;
                                    if battery_life < enough_battery_home || battery_life == enough_battery_home
                                        current_state = 4;
                                        fprintf('State: Returning Home')
                                        wallet = wallet +1;
                                    end
                            end
                     elseif battery_life == enough_battery_home
                         current_state = 4;
                         fprintf('State: Returning Home')
                         wallet = wallet +1;
                     end
                 end
            else
                current_state = 4;
                wallet = wallet +1;
                fprintf('State: Returning Home\n')
            end
    end
end

%% Charging
if current_state == 4 || battery_life == enough_battery_home
   current_state = 6;
   wallet = wallet + 1;
   fprintf('State: Charging\n')
   battery_life = 100;
   charging_iterations = charging_iterations +1;
end

%% Low Battery
if battery_life <= 1
    wallet = wallet -2;
    fprintf('Robot has lost $2 for low battery. \n');
end
%% Battery Dead
if battery_life <= 0
    fprintf('The battery of the robot has been depleted.\n')
    return
end
end
average_iterations = average_iterations + iteration;
counter = counter + 1;
true_average = average_iterations / counter;

% Please inquire if you plan to use this outside of the WVU REU Summer 2021
% Program. Contact danielvillarrealusa@gmail.com