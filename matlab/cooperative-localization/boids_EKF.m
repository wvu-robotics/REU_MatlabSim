% boids rules ekf sim
addpath 'C:\Users\Trevs\Desktop\REU swarm\matlab\boids-model-master'

%EKF Sim

numBots = 10;
boids_count = numBots;
dt = 1;
boids = [];
time = 10;

%% -----------------------------initalize robots with their given truths
boids=Boid.empty;
Ks = 1;
Ka = 1;
Kc = 1;
for i=1:boids_count
   boids(i)=Boid(rand*640/3,rand*360/3, Ks,Ka, Kc, boids_count,time);
end
%% --------------------------------SIMULATE boids
for t = 1:time
    for r = 1:numBots
        boids(r) = boids(r).flock(boids); %get accelleration
        boids(r) = boids(r).update; % update velocity, position, path, and set accel to zero
        
        %update laser measurement with noise
        measure = [];
        for L = 1:numBots
            measure = [measure, norm(boids(L).position(1:2)- boids(r).position(1:2))+ normrnd(0,.01,1,1) ];
        end
        boids(r).laser = [boids(r).laser;measure];
        
    end
end

%%--------------------------plot truth data
figure()
axis equal
for r = 1:numBots
    quiver(boids(r).path(:,1),boids(r).path(:,2),boids(r).path(:,3),boids(r).path(:,4),0,'r');
    hold on;
end

%%---------------------------RUN filter on generated data

X_i = zeros(3*numBots, 1);
X_dot = X_i;
Y_m = [];
for r = 1:numBots
    X_i(r) = boids(r).path(2,1);
    X_i(r+numBots) = boids(r).path(2,2);
    theta = atan2(boids(r).path(2,4), boids(r).path(2,3));
    X_i(r+2*numBots) = theta;
   
    vel = norm(boids(r).path(2,3:4));
    X_dot(r) = vel*cos(theta + dt*boids(r).path(5));
    X_dot(r+numBots) = vel*sin(theta + dt*boids(r).path(5));
    X_dot(r+2*numBots) = boids(r).path(2,5);    %may need to fix later
    
    Y_m = [Y_m; boids(r).laser(2,:)'];
    
end

quiver(X_i(1:numBots), X_i(numBots+1:2*numBots),X_dot(1:numBots), X_dot(numBots+1:2*numBots),0, 'b');
    hold on;

P_i = .01*eye(length(X_i));
Q = 1e-4*P_i;
R = 10*eye(length(Y_m));

for t = 2:10
    [X_f,P_f] = Central_swarm_EKF(X_i,P_i,Y_m, X_dot, dt,Q, R);
    
    X_i = X_f;
    P_i = P_f;
    
    Y_m = [];
    for r = 1:numBots
        theta = X_f(r+2*numBots);
        vel = norm(boids(r).path(2,3:4));
   
        X_dot(r) = vel*cos(theta + dt*boids(r).path(5));
        X_dot(r+numBots) = vel*sin(theta + dt*boids(r).path(5));
        X_dot(r+2*numBots) = boids(r).path(2,5);
        Y_m = [Y_m; boids(r).laser(t,:)'];
    end
        
   quiver(X_f(1:numBots), X_f(numBots+1:2*numBots),X_dot(1:numBots), X_dot(numBots+1:2*numBots),0, 'b');
    hold on;
    pause(2);
end
hold off;



