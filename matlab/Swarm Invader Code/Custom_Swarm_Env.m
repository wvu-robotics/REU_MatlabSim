numRobots = 50;
env = MultiRobotEnv(numRobots);

numTeams = 2;  
% Initialize poses randomly, and add bias to each "team"
poses = [10*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];
angleBias = 2*pi*(1:numRobots)/numTeams;
poses(1:2,:) = poses(1:2,:) + 2.5*[sin(angleBias);cos(angleBias)];

% Update the environment
    env(1:numRobots, poses);
    xlim([-12 12]);   % Without this, axis resizing can slow things down
    ylim([-12 12]);