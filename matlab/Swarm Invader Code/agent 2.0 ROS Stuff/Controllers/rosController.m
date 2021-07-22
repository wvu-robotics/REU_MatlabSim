function rosController(agent)
    agent.color = [1 0 0];
    global CURRENT_KEY_PRESSED 
    switch CURRENT_KEY_PRESSED
        case "w"
             agent.velocityControl = 0.3*[0,1];
        case "a" 
             agent.velocityControl = 0.3*[-1, 0];
        case "s"
            agent.velocityControl = 0.3*[0,-1];
        case "d"
             agent.velocityControl = 0.3*[1, 0];
        case "q"
            agent.velocityControl = 0.3*[-1, 1];
        case "e"
            agent.velocityControl = 0.3*[1, 1];
        case "shift" % works with left or right shift, couldn't specify left
            agent.velocityControl = 0.3*[-1, -1];
        case "c"
             agent.velocityControl = 0.3*[1, -1];
        case "j"
            Z = [0 0 1];
            P = [agent.pose - [0,0],0];
            T = cross(P,Z);
            T = 0.3*(T/norm(T)); % has 3 components, only need 2 for velocity
            agent.velocityControl = T(1:2);
        case "k"
            for mesAgents = agent.measuredAgents
                if mesAgents.getProperty('enemy')
                    x = objectFlow1(mesAgents.pose, [0,0],10);
                    agent.velocityControl= 0.3*(x/norm(x));
                end
            end
        case "z"
            agent.velocityControl=[0,0];
    end
end