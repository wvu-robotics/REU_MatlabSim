function testController5(Agent1)
Agent1.color = [0 1 0]; % green
maxvel = 5; % chase to make max vel [max vel, max vel] and min vel is [-max vel, -maxvel]
run('register_variables_for_function.m');
placeholdervar = evolving_variables_function(Agent1);
% placeholdervar is to satisfy the requirement of an output for the function
% this value will not be used. all variables that needs to be changed were
% changed within the function file using classes and built-in MATLAB
% function to "assignin" base workspace. 
%                                       Current State     Wallet
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
placeholdervar1 = state_machine_function(Agent1); % same explanation above
for i = 1:length(Agent1.measuredAgents)
    if Agent1.measuredAgents(i).getProperty('isEnemy') == true
        invader_detected = 1;
        assignin('base','invader_detected',invader_detected); % changes base workspace variable value
        Agent1.setProperty('invader_detected',invader_detected); % for use later in the state machine when deciding what to do next
        fprintf('Invader Detected\n');
    end
end
% run('state_machine.m');
for i = 1:length(Agent1.measuredAgents) % 2 since invader is Agent #1
    if Agent1.getProperty('current_state') == 0 || Agent1.getProperty('current_state') == 1
        Agent1.velocityControl= [0,0]; % stationary 
        if Agent1.getProperty('current_state') == 1
            fprintf('State: On Alert\n')
        elseif Agent1.getProperty('current_state') == 0
            fprintf('State: Idle\n')
        end
        
    elseif Agent1.getProperty('current_state') == 2
        
        if Agent1.measuredAgents(i).getProperty('isEnemy') == true
            maginvader = norm(Agent1.pose - Agent1.measuredAgents(i).pose);
            % magnitude of vector from agent location to invader 
            maggoal = norm(Agent1.pose - Agent1.goalPose); 
            % magnitude of vector from agent location to goal (home)
            maginvadergoal = norm(Agent1.measuredAgents(i).pose - Agent1.goalPose);
            % magnitude of vector from invader to target (home) 
            
            if maginvader > mapSize || maggoal < 5 || maginvadergoal < 5
                current_state = 5;
            else
                for n = 1:length(Agent1.measuredAgents)
                    if Agent1.measuredAgents(i).getProperty('isEnemy') == true 
                        invaderpos = Agent1.measuredAgents(i).pose;
                        x = objectFlow1(invaderpos, Agent1.pose,1000);
                        currentvel= 5*(x/norm(x)); % speed is 5, unit vector is from objectFlow
                        comp1 = currentvel(1);
                        comp2 = currentvel(2);
                        while comp1 > maxvel || comp2 > maxvel || comp1 < -maxvel || comp2 < -maxvel
                        comp1 = comp1/1.1; % divide both vector component by 1.1 each loop so long as 
                        comp2 = comp2/1.1; % at least one componenet is greater than max vel or less than min vel.
                        end
                    end
                end
                fprintf('State: In Pursuit\n')
            end
        end
        
    elseif Agent1.getProperty('current_state') == 3 % state machine should not be outputting this
       fprintf('ERROR\nThe state machine is outputting current_state = 3\n');
       
    elseif Agent1.getProperty('current_state') == 4
        Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
        fprintf('State: Returning Home\n')
        
    elseif Agent1.getProperty('current_state') == 5
        %Agent1.velocityControl = 5 * objectFlow2(Home, Agent1.pose,100,Agent1);
       for n = 1:length(Agent1.measuredAgents)
        if Agent1.measuredAgents(i).getProperty('isEnemy') == true 
           invaderpos = Agent1.measuredAgents(i).pose;
           currentvel = 5*objectFlow2(invaderpos, Agent1.pose, Home, numberOfAgents*(1000/3), Agent1);  % speed is 5, unit vector is from objectFlow
           comp1 = currentvel(1);
           comp2 = currentvel(2);
           while comp1 > maxvel || comp2 > maxvel || comp1 < -maxvel || comp2 < -maxvel
               comp1 = comp1/1.1; % divide both vector component by 1.1 each loop so long as
               comp2 = comp2/1.1; % at least one componenet is greater than max vel or less than min vel.
           end
           Agent1.velocityControl = [comp1 comp2]; 
        end
       end
%         y = norm(Agent1.goalPose - Agent1.pose);
%         if y >1 %(mapSize/(mapSize/(15*agentRadius)))
%             Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
%         else
%             Z = [0 0 1];
%             P = [Agent1.pose - Agent1.goalPose,0];
%             T = cross(P,Z);
%             T = 5*(T/norm(T)); % has 3 components, only need 2 for velocity
%             Agent1.velocityControl = T(1:2); % creates circular movement
%                                              % around home.
%         end

    elseif Agent1.getProperty('current_state') == 6
        Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
        y = norm(Agent1.goalPose - Agent1.pose);
        if y < 2 % if magnitude of vector from goal (home) to agent is 2 or less, then it's close enough to begin charging
            Agent1.velocityControl= 0;
            fprintf('State: Charging\n')
        end

    elseif Agent1.getProperty('current_state') == 7 
        x = randi(mapSize,1,2);
        Agent1.velocityControl= 5*(x/norm(x));
        fprintf('Status: Searching\n')
        % random walk around small area for searching 
        
    elseif Agent1.getProperty('current_state') == 8
        Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
        fprintf('State: Gathering\n')
        
        % set course for current position to back home and subtract battery
        % life from percentage required to go back home. also have it go
        % back to original position (essentially subtract battery for a
        % round trip home and back to original position)
    elseif Agent1.getProperty('current_state') == 9
        Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
        fprintf('State: Shutdown\n')
        fprintf('Robot died %.2f ft. from home.\n',outstanding_distance)
        % set course headed back home but only let it get as far as its
        % battery will take it. have it "die" when its battery reaches
        % zero.
    else
        fprintf('ERROR.\nCurrent_state is NOT WORKING!\n')
    end
    
    
% for i = 1:length(Agent1.measuredAgents)
%     run('state_machine.m');
%     if Agent1.measuredAgents(i).getProperty('isEnemy') == false
%         if current_state == 9
%             x = Home - Agent1.measuredAgents(i).pose;
%             Agent1.velocityControl= 25*(x/norm(x));
%         end
%     end
%     
%     if Agent1.measuredAgents(i).getProperty('isEnemy') == true
%         x = objectFlow1(Agent1.measuredAgents(i).pose, Agent1.pose,1000000);
%         Agent1.velocityControl= 5*(x/norm(x));
%         % x  = Agent1.pose - Agent1.measuredAgents(i).pose -0.5;
%     end
% end



%     if Agent1.measuredAgents(i).getProperty('isEnemy') == false
%         x  = [-15,10] - Agent1.measuredAgents(i).pose;
%         Agent1.velocityControl= 25*(x/norm(x));
%         if norm(x) <= 2
%           jj = jj+1;
%           Agent1.velocityControl= 0;
%         end
%     end
% end

end
