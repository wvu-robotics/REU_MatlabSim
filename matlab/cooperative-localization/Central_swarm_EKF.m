function [X_f,P_f] = Central_swarm_EKF(X_i,P_i,Y_m, X_dot, dt,Q, R)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

numBots = round(length(X_i)/3);

X_p = eye(numBots*3)*X_i + eye(numBots*3)*dt*X_dot;

P_p = eye(numBots*3)*P_i*eye(numBots*3)' + Q;

Y_p = [];

H = zeros(numBots,numBots*3);

for i = 1:numBots
       
    dx = X_p(1:numBots)- X_p(i);
    dy =  X_p(numBots+1:2*numBots) - X_p(i+numBots);
    r = sqrt(dx.^2 + dy.^2);
    
 
    dH_dx = dx./r;
    dH_dy = dy./r;
    for a = 1:numBots
       if r(a) == 0
           dH_dx(a) = 0;
           dH_dy(a) = 0;
       end
    end
    dH_dtheta = zeros(size(dH_dx));
    
    H(:,i) = dH_dx;
    H(:,i+numBots) = dH_dy;
    H(:,i+2*numBots) = dH_dtheta;
   
    Y_p = [Y_p;r];
    
end
H0 = H;
for i = 2:numBots
    H = [H;H0];
end
 
residual = Y_m - Y_p;
S = H*P_p*(H')+R;
K = P_p*H'*inv(S);
X_f = X_p + K*residual;
I = eye(3*numBots);
P_f = (I-K*H)*P_p;





end

