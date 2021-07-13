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
for i = 2:length(Agent1.measuredAgents)
    if current_state == 0 || current_state == 1 %revisit this one
        % issue random walk velocity
    elseif current_state == 2
        % implement doublet code
    elseif current_state == 3 %check in state machine about this one,
                              %might not be an output.
       % issue command to remain in place
    elseif current_state == 4
        % set course for current position to back home and subtract battery
        % life from percentage required to go back home
    elseif current_state == 5
        % issue command for random walk around home
    elseif current_state == 6
        % remove velocity and set battery_life to 100
    elseif current_state == 7 
        % random walk around small area for searching 
    elseif current_state == 8
        % set course for current position to back home and subtract battery
        % life from percentage required to go back home. also have it go
        % back to original position
    elseif current_state == 9
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