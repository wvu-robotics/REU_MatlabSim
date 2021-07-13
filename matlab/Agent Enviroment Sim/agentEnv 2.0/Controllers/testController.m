function testController(agent)   
   agent.color = [ 0 0 1]; 
%    if any(agent.calcIdealUnitVec)
%       agent.velocityControl = agent.calcIdealUnitVec;
%    end
   agent.velocityControl = [.1, .1];
   agent.heading =  -atan2(agent.velocityControl(2),agent.velocityControl(1));
%    idealUnit = agent.calcIdealUnitVec;
%    colorVec = [.5*idealUnit+.5,.5];
  
end

