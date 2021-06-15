%reverse_PF sim

numBots = 100;
range = 5; % detection range
length = 10; %spawning radius
dt = 1;      %time step
noise = .25; %gaussian noise variance
ROBOTS = [];

%% -----------------------------initalize robots with their given truths
for r = 1:numBots
    x = length*rand(1,1);
    y = length*rand(1,1);
    theta = 2*pi*rand(1,1);
    vel = 5*rand(1,1);
    robot = struct('pose', [x,y,theta], 'vel', vel, 'path', [x,y,theta], 'laser', zeros(1,numBots), 'bearing', zeros(1,numBots), 'particles', zeros(3,numBots));
    ROBOTS = [ROBOTS,robot];
end

%% ----------------------------simulate Robots
figure()
for t = 1:10
    for r = 1:numBots %our robot
        for L = 1:numBots % other robot
  
            %distance from r to L
            d = norm(ROBOTS(L).pose(1:2)- ROBOTS(r).pose(1:2))+ normrnd(0,noise,1,1);
            %angle from r to L
            phi = atan2(ROBOTS(L).pose(2)- ROBOTS(r).pose(2), ROBOTS(L).pose(1)- ROBOTS(r).pose(1))+ normrnd(0,noise,1,1);%+ normrnd(0,.01,1,1)
            
            %if the other robot is in detection / communication range
            %give it our dead-reckoning prediction of where it is
            %update our particle if we have one there already
            if d < range
                dx = d*cos(phi); %from r -> L
                dy = d*sin(phi); %from r -> L
                x0 = ROBOTS(r).pose(1); %r pose
                y0 = ROBOTS(r).pose(2); %r pose
                %give L, r's position of L, and mark that it has that
                %particle
                ROBOTS(L).particles(:,r) = [x0+dx;y0+dy;1];
            end
        end
    end
    
     
    %%  --------------plot agents' means and covariances
    clf()
    for r = 1:numBots
        plot(ROBOTS(r).pose(1),ROBOTS(r).pose(2),'ks');
        viscircles([ROBOTS(r).pose(1),ROBOTS(r).pose(2)],range);
        hold on;
        x_mean = mean(ROBOTS(r).particles(1,ROBOTS(r).particles(3,:) >.5));
        y_mean = mean(ROBOTS(r).particles(2,ROBOTS(r).particles(3,:) >.5));

        covar =  cov(ROBOTS(r).particles(1:2,ROBOTS(r).particles(3,:) >.5)');
        error_ellipse(covar, [x_mean,y_mean])
        hold on;
        plot(x_mean, y_mean, 'go')
        hold on;
    end
    axis([-50 50 -50 50])
    pause(.0001);
    
    %% -------------------------------update location of robots
    for r = 1:numBots
        dx = ROBOTS(r).vel*cos(ROBOTS(r).pose(3))*dt;
        dy = ROBOTS(r).vel*sin(ROBOTS(r).pose(3))*dt;
        ROBOTS(r).pose = ROBOTS(r).pose + [dx, dy ,0];
        ROBOTS(r).path = [ROBOTS(r).path; ROBOTS(r).pose];
        ROBOTS(r).particles(1,:) =  ROBOTS(r).particles(1,:) + dx+ normrnd(0,noise,1,numBots);
        ROBOTS(r).particles(2,:) =  ROBOTS(r).particles(2,:) + dy+ normrnd(0,noise,1,numBots);
        ROBOTS(r).pose = ROBOTS(r).pose;
    end
    
   
    
    
end

















