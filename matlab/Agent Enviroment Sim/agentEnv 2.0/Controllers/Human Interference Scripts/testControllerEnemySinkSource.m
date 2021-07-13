function testControllerEnemySinkSource(Agent1)
    Agent1.color = 'red'; %change color
    desired_Vel = [0,0]; %intiailizing default velocity
    %intialize a for loop for the number of agents
    for i = 1:length(Agent1.measuredAgents) 
        diffPose = Agent1.measuredAgents(i).pose - Agent1.pose ; %finds the diffrence vector between the agents
        diffPose = diffPose/norm(diffPose); %unit vector
        desired_Vel = desired_Vel - diffPose; %substracts vector of diffpose and adds it too the velocity
    end
    desired_Vel = desired_Vel + Agent1.calcIdealUnitVec*norm(Agent1.pose - Agent1.goalPose); %Adds the goal attraction force
  
    Agent1.velocityControl = 5*(desired_Vel/norm(desired_Vel)); %take unit vector multiply by five
    end

    