function testController(agent)
    agent.velocityControl = agent.calcIdealUnitVec;
   
   idealUnit = agent.calcIdealUnitVec;
   colorVec = [.5*idealUnit+.5,.5];
   agent.color = colorVec;
   
end

