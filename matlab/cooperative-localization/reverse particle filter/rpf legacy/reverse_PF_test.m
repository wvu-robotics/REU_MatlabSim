% reverse Particle filter

numBots = 10;
dt = 1;
ROBOTS = [];

%%-----------------------------initalize robots with their given truths
for r = 1:numBots
    x = 10*rand(1,1);
    y = 10*rand(1,1);
    theta = 2*pi*rand(1,1);
    vel = 5*rand(1,1);
    robot = struct('pose', [x,y,theta], 'vel', vel, 'path', [x,y,theta], 'laser', zeros(1,numBots), 'bearing', zeros(1,numBots), 'particles', zeros(3,numBots));
    ROBOTS = [ROBOTS,robot];
end

%%--------------------------------SIMULATE ROBOTS
for t = 1:10
    for r = 1:numBots
        ROBOTS(r).pose = ROBOTS(r).pose + [ROBOTS(r).vel*cos(ROBOTS(r).pose(3))*dt, ROBOTS(r).vel*sin(ROBOTS(r).pose(3))*dt,0];
        ROBOTS(r).path = [ROBOTS(r).path; ROBOTS(r).pose];
        ROBOTS(r).pose = ROBOTS(r).pose + normrnd(0,.01,1,3);
        
        measure = [];
        angles = [];
        for L = 1:numBots
            measure = [measure, norm(ROBOTS(L).pose(1:2)- ROBOTS(r).pose(1:2))+ normrnd(0,.01,1,1) ];
            angles = [angles, atan2(ROBOTS(L).pose(2)- ROBOTS(r).pose(2), ROBOTS(L).pose(1)- ROBOTS(r).pose(1))];
        end
        ROBOTS(r).laser = [ROBOTS(r).laser;measure];
        ROBOTS(r).bearing = [ROBOTS(r).bearing;angles];
        
    end
end

%%--------------------------plot truth data
% figure()
% axis equal
% for r = 1:numBots
%     quiver(ROBOTS(r).path(:,1),ROBOTS(r).path(:,2),ROBOTS(r).vel*cos(ROBOTS(r).pose(3))*ones(11,1), ROBOTS(r).vel*sin(ROBOTS(r).pose(3))*ones(11,1),0,'r');
%     hold on;
% end

t = 11;
for r = 2:numBots
    phi = ROBOTS(r).bearing(t,1);
    dx = ROBOTS(r).laser(t,1)*cos(phi);
    dy = ROBOTS(r).laser(t,1)*sin(phi);
    x0 = ROBOTS(r).pose(1);
    y0 = ROBOTS(r).pose(2);
    ROBOTS(1).particles(:,r) = [x0+dx;y0+dy;1];
    
end

x_mean = mean(ROBOTS(1).particles(1,2:end));
y_mean = mean(ROBOTS(1).particles(2,2:end));
covar =  cov(ROBOTS(1).particles(1:2,ROBOTS(1).particles(3,:) >.5)');

figure()
axis equal
plot(ROBOTS(1).particles(1,2:end), ROBOTS(1).particles(2,2:end), '*b');
hold on;
plot(ROBOTS(1).path(end,1), ROBOTS(1).pose(end,2), 'r^')
hold on;
plot(x_mean, y_mean, 'go')
hold on;
error_ellipse(covar, [x_mean,y_mean])
% for r = 1:numBots
%     plot(ROBOTS(r).pose(1),ROBOTS(r).pose(2),'ks');
%     hold on;
% end

error = norm([ROBOTS(1).pose(1)-x_mean,ROBOTS(1).pose(2)-y_mean])


