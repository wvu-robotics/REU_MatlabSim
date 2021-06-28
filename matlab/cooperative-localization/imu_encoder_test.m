
% imu_encoder test

t = 1:1:100;
dt = 1;

Vx_T = 0.1;
Vy_T = 0;
Theta_T = 0;
Omega_T = 0;
Ax_T = 0;
Ay_T = 0;

for k = 2:100
    Ax_T(k) = cos(k);
    Ay_T(k) = sin(k);
    Vx_T(k) = Ax_T(k) + Vx_T(k-1);
    Vy_T(k) = Ay_T(k) + Vy_T(k-1);
    Omega_T(k) = 1;
    Theta_T(k) = Omega_T(k) + Theta_T(k-1);
    
end





X_i = [Vx_T(1); Vy_T(1);Theta_T(1)];
P_i = .1*eye(3);
Q = .1*eye(3);
R = .1*eye(2);

measurment = [];
HIST = [];
for k = 1:100
    y_m = [norm([Vx_T(k),Vy_T(k)]);Omega_T(k)]+normrnd(0,.1,2,1);
    u = [Ax_T(k);Ay_T(k);Omega_T(k)]+normrnd(0,.1,3,1);
    [X_f,P_f] = imu_encoder_vel_EKF(X_i,P_i,y_m,u,Q,R,dt);
    HIST = [HIST,X_f];
    X_i = X_f;
    P_i = P_f;
    measurment = [measurment,y_m];
    

end

figure();
subplot(3,1,1)
plot(t,Vx_T);
hold on;
plot(t,HIST(1,:));
title('Vx')

subplot(3,1,2)
plot(t,Vy_T);
hold on;
plot(t,HIST(2,:));
title('Vy')

subplot(3,1,3)
plot(t,Theta_T);
hold on;
plot(t,HIST(3,:));
title('theta')











