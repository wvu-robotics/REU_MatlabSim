function testControllerEnemy1(Agent1)
    Agent1.color = 'red';
    for i=1:500
         x = Agent1.pose + randi([-8,8],1,2) - randi([-8,8],1,2) -5;
         %x = Agent1.pose + randi([0,1],1,2);
        Agent1.velocityControl= -5*(x/norm(x));
end