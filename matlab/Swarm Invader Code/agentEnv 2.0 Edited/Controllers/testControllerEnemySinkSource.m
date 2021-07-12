function testControllerEnemySinkSource(Agent1)
    Agent1.color = 'red';
    desired_Vel = [0,0];
    for i = 1:length(Agent1.measuredAgents) 
        diffPose = Agent1.measuredAgents(i).pose - Agent1.pose ;
        diffPose = diffPose/norm(diffPose);
        desired_Vel = desired_Vel - diffPose;
    end
    desired_Vel = desired_Vel + Agent1.calcIdealUnitVec*norm(Agent1.pose - Agent1.goalPose);
  
    Agent1.velocityControl = 5*(desired_Vel/norm(desired_Vel));
    end

    