function testControllerEnemy1(Agent1)
    Agent1.color = 'red';
   % Agent2.color = 'purple';
    for i=1:500
         x = Agent1.pose + randi([-8,10],1,2) - randi([-8,10],1,2) -5;
        % x = Agent2.pose + randi([-8,8],1,2) - randi([-8,8],1,2) -5;
         %x = Agent1.pose + randi([0,1],1,2);
        Agent1.velocityControl= -5*(x/norm(x));
         %Agent2.velocityControl= -5*(x/norm(x));
    end

    