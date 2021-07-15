function rosController(agent)
    agent.color = [1 0 0];
    global CURRENT_KEY_PRESSED 
    switch CURRENT_KEY_PRESSED
        case "w"
            agent.velocityControl = 7*[0,1];
        case "a" 
            agent.velocityControl = 7*[-1, 0];
        case "s"
            agent.velocityControl = 7*[0,-1];
        case "d"
            agent.velocityControl = 7*[1, 0];
        case "t"
            agent.velocityControl = 7*[-1, 1];
        case "y"
            agent.velocityControl = 7*[1, 1];
        case "g"
            agent.velocityControl = 7*[-1, -1];
        case "h"
            agent.velocityControl = 7*[1, -1];
        case "c"
            Z = [0 0 1];
            P = [agent.pose - [0,0],0];
            T = cross(P,Z);
            T = 5*(T/norm(T)); % has 3 components, only need 2 for velocity
            agent.velocityControl = T(1:2);
        case "k"
            x = objectFlow1([1,1], agent.pose,1000000);
            agent.velocityControl= 7*(x/norm(x));
        case "z"
            agent.velocityControl=0;
    end
end