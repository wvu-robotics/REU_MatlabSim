function [X_f,P_f] = imu_encoder_vel_EKF(X_i,P_i,y_m,u,Q,R,t)
%UNTITLED Summary of this function goes here
%
%
%     X   =       A       *   X   +       B       *    u
%     
%   | Vx|   | 1   0   0 |   | Vx|   | t   0   0 |   | Ax    |
%   | Vy| = | 0   1   0 | * | Vy| + | 0   t   0 | * | Ay    |
%   |Yaw|   | 0   0   1 |   |Yaw|   | 0   0   t |   | omega |
%
%
%      Y       =          C*X         +     D     *   u      
%     
%   | V     |  = | sqrt(Vx^2 +Vy^2) | + | 0 0 0 | * | Ax    |
%   | omega |    |        0         |   | 0 0 1 |   | Ay    |
%                                                   | omega |
%
%

 
F = eye(3);
V_i = norm(X_i(1:2));
 
H = [X_i(1)/V_i, X_i(2)/V_i, 0;
        0,          0,       0];
 
I = eye(3);
X_p = X_i + u*t;
P_p = F*P_i*F' + Q;
V_p = norm(X_p(1:2));
y_p = [V_p;u(3)];

residual = y_m - y_p;
S = H*P_p*H'+R;
K = P_p*H'*inv(S);
X_f = X_p + K*residual;
P_f = (I-K*H)*P_p;



end

