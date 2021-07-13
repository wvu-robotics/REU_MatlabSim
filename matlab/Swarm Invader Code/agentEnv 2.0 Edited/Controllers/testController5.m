function testController5(Agent1)
Agent1.color = [0 1 0];
run('register_variables_for_function.m');
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
run('state_machine.m');
current_state = 2;
% Agent1.setProperty("Battery_Life",battery_life);
% Agent1.setProperty("Distance_From_Home",distance_from_home);
% Agent1.setProperty("Distance_From_Invader",distance_from_invader);
% Agent1.getProperty("Battery_Life");
% Agent1.getProperty("Distance_From_Home");
% Agent1.getProperty("Distance_From_Invader");
for i = 2:length(Agent1.measuredAgents) % 2 since invader is Agent #1
    
    if current_state == 0 || current_state == 1 %revisit this one
        
        Agent1.velocityControl= 0;
        
    elseif current_state == 2
        
        if Agent1.measuredAgents(i).getProperty('isEnemy') == true
%             maginvader = norm(Agent1.pose - Agent1.measuredAgents(i).pose);
%             maggoal = norm(Agent1.pose - Agent1.goalPose); 
%             maginvadergoal = norm(Agent1.measuredAgents(i).pose - Agent1.goalPose);
%             
%             if maginvader > 35 || maggoal < 5 || maginvadergoal < 10
%                 current_state = 5;
%             else
                x = objectFlow1(Agent1.measuredAgents(i).pose, Agent1.pose,1000000)+[0.5,0.5];
                Agent1.velocityControl= 5*(x/norm(x));
%             end
        end
        
    elseif current_state == 3 % state machine should not be outputting this
       fprintf('ERROR\nThe state machine is outputting current_state = 3\n');
       
    elseif current_state == 4
        Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
        
    elseif current_state == 5
        y = norm(Agent1.goalPose - Agent1.pose);
        if y > 4
            Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
        else
            Z = [0 0 1];
            P = [Agent1.pose - Agent1.goalPose,0];
            T = cross(P,Z);
            T = 5*(T/norm(T)); % has 3 components, only need 2 for velocity
            Agent1.velocityControl = T(1:2); % creates circular movement
                                             % around home.
        end

    elseif current_state == 6
        Agent1.velocityControl= 0;

    elseif current_state == 7 
        x = Agent1.pose + randi([-8,10],1,2) - randi([-8,10],1,2) -5;
        Agent1.velocityControl= 5*(x/norm(x));
        % random walk around small area for searching 
        
    elseif current_state == 8
%         jj = [5,5];
        Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
%         Agent1.velocityControl = 5*(jj/norm(jj));
        % set course for current position to back home and subtract battery
        % life from percentage required to go back home. also have it go
        % back to original position
    elseif current_state == 9
        Agent1.velocityControl = Agent1.calcIdealUnitVec * 5;
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