function testController5(Agent1)
Agent1.color = 'green';
for i = 1:length(Agent1.measuredAgents)
   if Agent1.measuredAgents(i).getProperty('l') == true
       x = objectFlow1(Agent1.measuredAgents(i).pose, Agent1.pose,0.001);
      Agent1.velocityControl= -10*(x/norm(x));
      % x  = Agent1.pose - Agent1.measuredAgents(i).pose -0.5;
   end
end
end