
function rosController(agent) 
agent.color = [1 0 0];
    global CURRENT_KEY_PRESSED 
    switch CURRENT_KEY_PRESSED
        case "uparrow"
            agent.velocityControl = .5*[0,1];
        case "downarrow"
            agent.velocityControl = .5*[0,-1];
        case "rightarrow"
            agent.velocityControl = .5*[1, 0];
        case "leftarrow" 
            agent.velocityControl = .5*[-1, 0];
        case "a"
            agent.angularVelocityControl = .5;
         case "d"
            agent.angularVelocityControl = -.5;
        otherwise
            agent.velocityControl = [0,0];
            agent.angularVelocityControl = 0;
            
    end
end