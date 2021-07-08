function testController(agent) 
   if ~isempty(agent.measuredAgents)
       agent.color = 'r';
        for i = 1:length(agent.measuredAgents)
            x = agent.pose - agent.measuredAgents(i).pose;
            x = x/norm(x);
            agent.velocityControl = 5*x;
        end 
   else
       agent.velocityControl = 0.1*[1 1]; 
       agent.color = 'g';
   end
   agent.velocity
  
   
end

