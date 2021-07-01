function testController5(Agent1)
Agent1.color = 'green';
for i = 1:length(Agent1.measuredAgents)
   if Agent1.measuredAgents(i).getProperty('l') == true
      x  = Agent1.pose - Agent1.measuredAgents(i).pose -0.5;
      Agent1.velocityControl= -5*(x/norm(x));
   end
end
end