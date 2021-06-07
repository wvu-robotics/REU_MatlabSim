
%EKF Sim

numBots = 10;
dt = 1;
ROBOTS = [];

%%-----------------------------initalize robots with their given truths
for r = 1:numBots
    x = 10*rand(1,1);
    y = 10*rand(1,1);
    theta = 2*pi*rand(1,1);
    vel = 5*rand(1,1);
    robot = struct('pose', [x,y,theta], 'vel', vel, 'path', [x,y,theta], 'laser', zeros(1,numBots));
    ROBOTS = [ROBOTS,robot];
end

%%--------------------------------SIMULATE ROBOTS
for t = 1:10
    for r = 1:numBots
        ROBOTS(r).pose = ROBOTS(r).pose + [ROBOTS(r).vel*cos(ROBOTS(r).pose(3))*dt, ROBOTS(r).vel*sin(ROBOTS(r).pose(3))*dt,0];
        ROBOTS(r).path = [ROBOTS(r).path; ROBOTS(r).pose];
        
        measure = [];
        for L = 1:numBots
            measure = [measure, norm(ROBOTS(L).pose(1:2)- ROBOTS(r).pose(1:2))+ normrnd(0,.01,1,1) ];
        end
        ROBOTS(r).laser = [ROBOTS(r).laser;measure];
        
    end
end

%%--------------------------plot truth data
figure()
axis equal
for r = 1:numBots
    quiver(ROBOTS(r).path(:,1),ROBOTS(r).path(:,2),ROBOTS(r).vel*cos(ROBOTS(r).pose(3))*ones(11,1), ROBOTS(r).vel*sin(ROBOTS(r).pose(3))*ones(11,1),0,'r');
    hold on;
end

%%---------------------------RUN filter on generated data

X_i = zeros(3*numBots, 1);
X_dot = X_i;
Y_m = [];
for r = 1:numBots
    X_i(r) = ROBOTS(r).path(2,1);
    X_i(r+numBots) = ROBOTS(r).path(2,2);
    X_i(r+2*numBots) = ROBOTS(r).path(2,3);
   
    X_dot(r) = ROBOTS(r).vel*cos(ROBOTS(r).pose(3));
    X_dot(r+numBots) = ROBOTS(r).vel*sin(ROBOTS(r).pose(3));
    X_dot(r+2*numBots) = 0;
    
    Y_m = [Y_m; ROBOTS(r).laser(2,:)'];
    
end

quiver(X_i(1:numBots), X_i(numBots+1:2*numBots),X_dot(1:numBots), X_dot(numBots+1:2*numBots),0, 'b');
    hold on;

P_i = .01*eye(length(X_i));
Q = 1e-4*P_i;
R = 10*eye(length(Y_m));

for t = 2:10
    [X_f,P_f] = Central_swarm_EKF(X_i,P_i,Y_m, X_dot, dt,Q, R);
    
    %X_dot = (X_f-X_i)/dt;
    X_i = X_f;
    P_i = P_f;
    
    Y_m = [];
    for r = 1:numBots
        Y_m = [Y_m; ROBOTS(r).laser(t,:)'];
    end
        
   quiver(X_f(1:numBots), X_f(numBots+1:2*numBots),X_dot(1:numBots), X_dot(numBots+1:2*numBots),0, 'b');
    hold on;
    pause(2);
end
hold off;


