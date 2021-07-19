function rosControllerDefender(agent) 
agent.color = [0 1 0];
    global CURRENT_KEY_PRESSED 
    switch CURRENT_KEY_PRESSED
        case "uparrow"
            agent.velocityControl = 10*[0,1];
        case "downarrow"
            agent.velocityControl = 10*[0,-1];
        case "rightarrow"
            agent.velocityControl = 10*[1, 0];
        case "leftarrow" 
            agent.velocityControl = 10*[-1, 0];
         case "t"
            agent.velocityControl = 10*[-1, 1];
        case "y"
            agent.velocityControl = 10*[1, 1];
        case "g"
            agent.velocityControl = 10*[-1, -1];
        case "h"
            agent.velocityControl = 10*[1, -1];
        case "c"
            Z = [0 0 1];
            P = [agent.pose - [0,0],0];
            T = cross(P,Z);
            T = 1.5*(T/norm(T)); % has 3 components, only need 2 for velocity
            agent.velocityControl = T(1:2);
        case "k"
            x = objectFlow1([1,1], agent.pose,1000000);
            agent.velocityControl= 5*(x/norm(x));
    end
end