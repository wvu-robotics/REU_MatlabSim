function testController(agent)   

   if any(agent.calcIdealUnitVec)
      agent.velocityControl = 5*agent.calcIdealUnitVec;
   end
   desiredHeading =  -atan2(agent.velocityControl(2),agent.velocityControl(1));
   agent.heading = desiredHeading;
   
   idealUnit = agent.calcIdealUnitVec;
   colorVec = [.5*idealUnit+.5,.5];
   agent.color = colorVec;
end
