% This function file was made from the state machine script. The only
% changes are that it now incorporates the separate properties. All
% comments can be found in the original state machine script. To summarize
% it: the state machine takes in individual, evolving variables and static
% variables to allow robots to make decisions. The decisions can be seen by
% observing the "current state", which is what will enable robots to take
% up different actions to complete the task they choose to. "set" "get"
% Property is to allow the robots to set the value of their own individual
% private properties. the ENV class and Agent class are critical in
% ensuring this runs smoothly. 
function [current_state] = state_machine_function(Agent)
run('register_variables_for_function.m');
for i = 1:length(Agent.measuredAgents)
    if Agent.measuredAgents(i).getProperty('isEnemy') == false
        %% Battery Check
        if Agent.getProperty("Battery_Life") < Agent.getProperty("Enough_Battery_Home")
            current_state = 9;
            assignin('base','current_state',current_state);
            Agent.setProperty('current_state',current_state);
            battery_life = Agent.getProperty('battery_life') - Agent.getProperty('enough_battery_home');
            assignin('base','battery_life',battery_life);
            Agent.setProperty('battery_life',battery_life);
            outstanding_distance = abs(4*mapSize*Agent.getProperty('battery_life')); % 4ft/1% * battery life = ft
            assignin('base','outstanding_distance',outstanding_distance);
            Agent.setProperty('outstanding_distance',outstanding_distance);
            battery_life = 0;
            assignin('base','battery_life',battery_life);
            Agent.setProperty('battery_life');
            wallet = Agent.getProperty('wallet') - 2;
            assignin('base','wallet',wallet);
            Agent.setProperty('wallet',wallet);
            return
        
        elseif battery_life == enough_battery_home
            current_state = 4;
            assignin('base','current_state',current_state);
            Agent.setProperty('current_state',current_state);
            wallet = Agent.getProperty('wallet') + 1;
            assignin('base','wallet',wallet);
            Agent.setProperty('wallet',wallet);
        end
        %% Detect Invader
        if Agent.getProperty('invader_detected') == 1 % There has been an invader detected if == 1.
            if Agent.getProperty('battery_life') >= Agent.getProperty('enough_battery_home_invader')
                          % Checking to see if their battery life is enough
                          % to chase invader and get back home.
                current_state = 1; % If they have enough battery, they are on alert.
                assignin('base','current_state',current_state);
                Agent.setProperty('current_state',current_state);
                wallet = Agent.getProperty('wallet') + 2;
                assignin('base','wallet',wallet);
                Agent.setProperty('wallet',wallet);
            else
                current_state = 4; % If their battery is below 50%
                                   % then their current state is 
                                   % returning home.
                assignin('base','current_state',current_state);
                Agent.setProperty('current_state',current_state);
                wallet = Agent.getProperty('wallet') + 1;
                assignin('base','wallet',wallet);
                Agent.setProperty('wallet',wallet);
                battery_life = Agent.getProperty('battery_life') - Agent.getProperty('enough_battery_home');
                assignin('base','battery_life',battery_life);
                Agent.setProperty('battery_life',battery_life);
            end
            
        elseif Agent.getProperty('battery_life') >= Agent.getProperty('enough_battery_home')
            current_state = 0; % If invader_detected == 0 then their
                               % current state is 0. 
            assignin('base','current_state',current_state);
            Agent.setProperty('current_state',current_state);
            wallet = Agent.getProperty('wallet') + 1;
            assignin('base','wallet',wallet);
            Agent.setProperty('wallet',wallet);
        else
            current_state = 4;
            assignin('base','current_state',current_state);
            Agent.setProperty('current_state',current_state);
            wallet = Agent.getProperty('wallet') + 1;
            assignin('base','wallet',wallet);
            Agent.setProperty('wallet',wallet);
            battery_life = Agent.getProperty('battery_life') - Agent.getProperty('enough_battery_home');
            assignin('base','battery_life',battery_life);
            Agent.setProperty('battery_life',battery_life);
        end
        %% Invader Detected
        if Agent.getProperty('current_state') == 1 % Battery life has been confirmed to be enough to go
                              % to invader and back home. Now they are deciding what to do
                              % to defend against the invader
            if Agent.getProperty('distance_from_home') <= 50 % Robots close to home will return home to
                                        % protect it.
                difference_invader_home = abs(Agent.getProperty('distance_from_invader') - Agent.getProperty('distance_from_home'));
                
                if Agent.getProperty('distance_from_invader') <= Agent.getProperty('distance_from_home') || difference_invader_home <= 5 
                   current_state = 2;
                   assignin('base','current_state',current_state);
                   Agent.setProperty('current_state',current_state);
                   wallet = Agent.getProperty('wallet') +5;
                   assignin('base','wallet',wallet);
                   Agent.setProperty('wallet',wallet);
                else
                    current_state = 4;
                    assignin('base','current_state',current_state);
                    Agent.setProperty('current_state',current_state);
                    wallet = Agent.getProperty('wallet') + 1;
                    assignin('base','wallet',wallet);
                    Agent.setProperty('wallet',wallet);
                    battery_life = Agent.getProperty('battery_life') - Agent.getProperty('enough_battery_home');
                    assignin('base','battery_life',battery_life);
                    Agent.setProperty('battery_life',battery_life);
                end
            else
                current_state = 2; % If distance_from_home > 50, then they will
                                   % pursue the invader
                assignin('base','current_state',current_state);
                Agent.setProperty('current_state',current_state);
                wallet = Agent.getProperty('wallet') + 5;
                assignin('base','wallet',wallet);
                Agent.setProperty('wallet',wallet);
            end
        end
        %% No Invader Detected
        if Agent.getProperty('current_state') == 0 % If there is no invader detected then they
                              % begin mapping their choices while idle.
            if Agent.getProperty('battery_life') >= (Agent.getProperty('enough_battery_home') +5) % If battery life is enough to
                                                        % get home then they plan their
                                                        % next move
                current_state = 3;
                assignin('base','current_state',current_state);
                Agent.setProperty('current_state',current_state);
                wallet = Agent.getProperty('wallet') +1;
                assignin('base','wallet',wallet);
                Agent.setProperty('wallet',wallet);
                deciding_number = rand(1)*(1-0)+0; %random number between [0,1]
                
                if deciding_number >= 0.5
                    current_state = 7;
                    assignin('base','current_state',current_state);
                    Agent.setProperty('current_state',current_state);
                    wallet = Agent.getProperty('wallet') + 2;
                    assignin('base','wallet',wallet);
                    Agent.setProperty('wallet',wallet);
                    
                    for dit = 0:10
                        if Agent.getProperty('battery_life') >= (Agent.getProperty('enough_battery_home') +1)
                            battery_life = Agent.getProperty('battery_life') - 1;
                            assignin('base','battery_life',battery_life);
                            Agent.setProperty('battery_life',battery_life);
                         
                            searching_iterations = Agent.getProperty('searching_iterations') +1;
                            assignin('base','searching_iterations',searching_iterations);
                            Agent.setProperty('searching_iterations',searching_iterations);
                            foraging_number = rand(1)*(1-0)+0;
                            
                            if foraging_number >= 0.95 && Agent.getProperty('battery_life') >= Agent.getProperty('enough_battery_home')
                                current_state = 8;
                                assignin('base','current_state',current_state);
                                Agent.setProperty('current_state',current_state);
                                wallet = Agent.getProperty('wallet') + 5;
                                assignin('base','wallet',wallet);
                                Agent.setProperty('wallet',wallet);
                                battery_life = Agent.getProperty('battery_life') - (2*Agent.getProperty('enough_battery_home'));
                                assignin('base','battery_life',battery_life);
                                Agent.setProperty('battery_life',battery_life);
                                gathering_iterations = Agent.getProperty('gathering_iterations') + 1;
                                assignin('base','gathering_iterations',gathering_iterations);
                                Agent.setProperty('gathering_iterations',gathering_iterations);
                                
                                if Agent.getProperty('battery_life') < Agent.getProperty('enough_battery_home') || Agent.getProperty('battery_life') == Agent.getProperty('enough_battery_home')
                                    current_state = 4;
                                    assignin('base','current_state',current_state);
                                    Agent.setProperty('current_state',current_state);
                                    wallet = Agent.getProperty('wallet') +1;
                                    assignin('base','wallet',wallet);
                                    Agent.setProperty('wallet',wallet);
                                end
                            end
                        elseif Agent.getProperty('battery_life') == Agent.getProperty('enough_battery_home')
                            current_state = 4;
                            assignin('base','current_state',current_state);
                            Agent.setProperty('current_state',current_state);
                            wallet = Agent.getProperty('wallet') + 1;
                            assignin('base','wallet',wallet);
                            Agent.setProperty('wallet',wallet);
                        end
                    end
                else
                    current_state = 4;
                    assignin('base','current_state',current_state);
                    Agent.setProperty('current_state',current_state);
                    wallet = Agent.getProperty('wallet') + 1;
                    assignin('base','wallet',wallet);
                    Agent.setProperty('wallet',wallet);
                end
            end
        end
        %% Charging
        if Agent.getProperty('current_state') == 4 || Agent.getProperty('battery_life') == Agent.getProperty('enough_battery_home')
            current_state = 6;
            assignin('base','current_state',current_state);
            Agent.setProperty('current_state',current_state);
            wallet = Agent.getProperty('wallet') + 1;
            assignin('base','wallet',wallet);
            Agent.setProperty('wallet',wallet);
            battery_life = 100;
            assignin('base','battery_life',battery_life);
            Agent.setProperty('battery_life',battery_life);
            charging_iterations = Agent.getProperty('charging_iterations') +1;
            assignin('base','charging_iterations',charging_iterations);
            Agent.setProperty('charging_iterations',charging_iterations);
        end
        %% Low Battery
        if Agent.getProperty('battery_life') <= 1
            wallet = Agent.getProperty('wallet') - 2;
            assignin('base','wallet',wallet);
            Agent.setProperty('wallet',wallet);
            fprintf('Robot has lost $2 for low battery. \n');
        end
        %% Battery Dead
        if Agent.getProperty('battery_life') <= 0
            fprintf('The battery of the robot has been depleted.\n')
            return
        end
    end
end
end

% Any questions can be directed to danielvillarrealusa@gmail.com. 