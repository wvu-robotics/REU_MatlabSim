function testController5(Agent1)
Agent1.color = [0 1 0];
for i = 1:length(Agent1.measuredAgents)
   if Agent1.measuredAgents(i).getProperty('isEnemy') == true
       x = objectFlow1(Agent1.measuredAgents(i).pose, Agent1.pose,1);
      Agent1.velocityControl= 5*(x/norm(x));
      % x  = Agent1.pose - Agent1.measuredAgents(i).pose -0.5;
   end
end
end