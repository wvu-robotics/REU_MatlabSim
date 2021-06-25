function testController(agent)
   
   for i = 1:length(agent.measuredAgents)
       agent.pose = agent.pose +.01*(agent.pose - agent.measuredAgents(i).pose);
   end 
   idealUnit = agent.calcIdealUnitVec;
   colorVec = [.5*idealUnit+.5,.5];
   agent.color = colorVec;
   agent.pose = agent.pose + .01;
end

