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
        case "q"
            agent.velocityControl = 7*[-1, 1];
        case "e"
            agent.velocityControl = 7*[1, 1];
        case "shift" % works with left or right shift, couldn't specify left
            agent.velocityControl = 7*[-1, -1];
        case "c"
             agent.velocityControl = 7*[1, -1];
        case "j" % circular motion around point [0,0]
            Z = [0 0 1]; % need three components for cross product 
            P = [agent.pose - [0,0],0]; % need two componenets 
            T = cross(P,Z);
            T = 7*(T/norm(T)); % has 3 components, only need 2 for velocity
            agent.velocityControl = T(1:2); % only use first and second componenets
        case "k"
            for mesAgents = agent.measuredAgents
                if mesAgents.getProperty('enemy')
                    x = objectFlow1(mesAgents.pose, [0,0],10);
                    agent.velocityControl= 7*(x/norm(x));
                end
            end
        case "z"
            agent.velocityControl=[0,0]; % stationary 
    end
end