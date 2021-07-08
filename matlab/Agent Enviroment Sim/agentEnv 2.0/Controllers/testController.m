function testController(agent)
   agent.velocityControl = agent.calcIdealUnitVec;
   agent.heading =  -atan2(agent.velocityControl(2),agent.velocityControl(1));
   idealUnit = agent.calcIdealUnitVec;
   colorVec = [.5*idealUnit+.5,.5];
   agent.color = colorVec;
end

