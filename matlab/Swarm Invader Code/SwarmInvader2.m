% Create robot coordinates (xy)
n = 8; 
xy = rand(n,2)*.5+.25; % [x,y] coordinates, nx2
% Create invader coordinates (xyi)
xyi = [.5, 1.1]; % [x,y] coordinates, 1x2
% Compute vector components
u = xyi(1) - xy(:,1);  % <------- compute u
v = xyi(2) - xy(:,2);  % <------- compute v
% Plot results
figure()
tiledlayout(1,2)
nexttile
hold on
plot(xy(:,1), xy(:,2), 'bo')
plot(xyi(1), xyi(2), 'rx')
quiver(xy(:,1), xy(:,2), u, v)
xlim([0, 1])
ylim([-.5,1.2])
grid on
title('Scaled quivers')
nexttile
hold on
plot(xy(:,1), xy(:,2), 'bo')
plot(xyi(1), xyi(2), 'rx')
quiver(xy(:,1), xy(:,2), u, v, 'off')
xlim([0, 1])
ylim([-.5,1.2])
grid on
title('Not scaled')