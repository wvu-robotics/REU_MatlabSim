function testController4(agent)
agent.color = 'green';
for i = 1:length(agent.measuredAgents)
   if agent.measuredAgents(i).getProperty('l') == true
      x  = agent.pose - agent.measuredAgents(i).pose;
      agent.velocityControl= -5*(x/norm(x));
   end
end
end
