%% demo world Example
%Trevor Smith

%% Create a multi-robot environment
numRobots = 10;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;
env.robotRadius = 0.25;
load exampleMap
env.mapName = 'map';

%% Create and  add sensors for each Swarmbot
SWARMBOTS = cell(1,numRobots);
LIDARS = cell(1,numRobots);
ROBDECTS = cell(1,numRobots); 

for r = 1:numRobots
    
    swarmbot = Swarmbot(r);
    SWARMBOTS{r} = swarmbot;
    
    lidar = MultiRobotLidarSensor;
    lidar.robotIdx = r;
    lidar.scanAngles = linspace(-pi,pi,25);
    lidar.maxRange = 4;
    attachLidarSensor(env,lidar);
    LIDARS{r} = lidar;
    
    detector = RobotDetector(env,r);
    detector.maxDetections = numRobots;
    detector.maxRange = 10;
    detector.fieldOfView = 2*pi;
    ROBDECTS{r} = detector;
end
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals


%% Animate and show the detections [range, angle, index]
poses = 4*(rand(3,numRobots).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];
env.Poses = poses;
ranges = cell(1,numRobots);
vel = .1*ones(3,numRobots);
neighbors = cell(1,numRobots);

for idx = 1:100
    % Get the current time step's sensor values
    
    for r = 1:numRobots
        scans = LIDARS{r}();
        ranges{r} = scans;
        detectedRobots = step(ROBDECTS{r}); 
        
        for a = 1:length(detectedRobots)
                neighbors{a} = SWARMBOTS{detectedRobots(a,3)};
        end
        
        % Update the Swarmbots with enviorment information
        SWARMBOTS{r} = SWARMBOTS{r}.getSensorData(vel(:,r), ranges{r}, neighbors);
        
        % update the enviornment with the Swarmbot action
        poses(:,r) = poses(:,r) + SWARMBOTS{r}.actOnEnv().velocity;
        
        env(1:numRobots, poses, ranges)
        
    end
    
    
end