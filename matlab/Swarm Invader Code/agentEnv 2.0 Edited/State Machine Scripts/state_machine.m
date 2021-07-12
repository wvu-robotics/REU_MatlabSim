%% Battery Check
if battery_life < enough_battery_home
    current_state = 9;
    fprintf('State: Shutdown\n')
    battery_life = battery_life - enough_battery_home;
    outstanding_distance = abs(4*mapSize*battery_life); % 4ft/1% * battery life = ft
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