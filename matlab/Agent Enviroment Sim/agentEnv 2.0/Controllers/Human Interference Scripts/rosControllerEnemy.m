function rosControllerEnemy(agent) 
agent.color = [1 0 0];
    global CURRENT_KEY_PRESSED 
    switch CURRENT_KEY_PRESSED
        case "uparrow"
            agent.velocityControl = 7*[0,1];
        case "downarrow"
            agent.velocityControl = 7*[0,-1];
        case "rightarrow"
            agent.velocityControl = 7*[1, 0];
        case "leftarrow" 
            agent.velocityControl = 7*[-1, 0];
    end
end