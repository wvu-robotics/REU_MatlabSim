function testControllerEnemySinkSource2(Agent1)
    Agent1.color = [1 0 0];%change color
    desired_Vel = [0,0];  %intiailizing default velocity
    k = 20000; %strength costant
    %intialize a for loop for the number of agents
    for i = 1:length(Agent1.measuredAgents) 
        diffPose = Agent1.measuredAgents(i).pose - Agent1.pose ; %%finds the diffrence vector between the agents
        desired_Vel = desired_Vel - k*diffPose./norm(diffPose).^3; %modeled by electrons f = e(qq/r^2) -> f = k/r^2
    end
    desired_Vel = desired_Vel + 10*Agent1.calcIdealUnitVec*norm(Agent1.pose - Agent1.goalPose).^2;  %Adds the goal attraction force
  
    Agent1.velocityControl = 5*(desired_Vel/norm(desired_Vel)); 
end