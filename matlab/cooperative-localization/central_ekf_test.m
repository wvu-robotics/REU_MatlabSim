


numBots = 3;
X_i = [0;1;2;0;1;2;0;1;2];
P_i = .01*eye(length(X_i));
Y_m = [0;1.4142;2.8284;1.4142;0;1.4142;2.8284;1.4142;0] + .01*(2*rand(9,1)-1);
X_dot = zeros(length(X_i),1);
dt =.01;
Q = 1e-4*P_i;
R = 1*eye(3*numBots);

X_0 = X_i;
figure()
plot(X_0(1:numBots), X_0(numBots+1:2*numBots), 'r');
hold on;
plot(X_f(1:numBots), X_f(numBots+1:2*numBots), 'b');

for t = 1:100
    [X_f,P_f] = Central_swarm_EKF(X_i,P_i,Y_m, X_dot, dt,Q, R);
    X_dot = (X_f-X_i)/dt;
    X_i = X_f;
    P_i = P_f;
    plot(X_f(1:numBots), X_f(numBots+1:2*numBots), 'b');
    hold on;
    %pause(2);
end

