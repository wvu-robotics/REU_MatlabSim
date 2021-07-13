function testControllerEnemySinkSource2(Agent1)
    Agent1.color = 'red';
    desired_Vel = [0,0];
    k = 50;
    for i = 1:length(Agent1.measuredAgents) 
        diffPose = Agent1.measuredAgents(i).pose - Agent1.pose ;
        desired_Vel = desired_Vel - k*diffPose./norm(diffPose).^3;
    end
    desired_Vel = desired_Vel + 10*Agent1.calcIdealUnitVec*norm(Agent1.pose - Agent1.goalPose).^2;
  
    Agent1.velocityControl = 5*(desired_Vel/norm(desired_Vel));
    end

    