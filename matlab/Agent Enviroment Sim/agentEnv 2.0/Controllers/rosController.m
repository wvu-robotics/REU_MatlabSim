function rosController(agent) 
agent.color = [1 0 0];
    global CURRENT_KEY_PRESSED 
    switch CURRENT_KEY_PRESSED
        case "uparrow"
            agent.velocityControl = 2*[0,1];
        case "downarrow"
            agent.velocityControl = 2*[0,-1];
        case "rightarrow"
            agent.velocityControl = 2*[1, 0];
        case "leftarrow" 
            agent.velocityControl = 2*[-1, 0];
    end
end