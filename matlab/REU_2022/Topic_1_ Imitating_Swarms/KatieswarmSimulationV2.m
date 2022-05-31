close all 
clear
clc

%% instantiate variables and robots
i = 0;
k = 4;
robot = zeros(4,2);

while (i < k)
    robot(i+1,1) = rand(1),rand(1);
    robot(i+1,2) = rand(1),rand(1);
    i = i+1;
end

%% define distance vector
robot_radius = .40;
i = 1;
k = 4;
L = 1;
m = 1;

for j = 1:length(robot)
    for k = 1:length(robot)
        if (robot(j,1)-robot(k,1) ~= 0)
            xdist(i) = robot(j,1)-robot(k,1);
            i = i+1;
        end
        if (robot(j,2)-robot(k,2) ~= 0)
            ydist(L) = robot(j,2)-robot(k,2);
            L = L+1;
        end
    end
end
    
horzdist = abs(xdist)';
vertdist = abs(ydist)';
horzdist = unique(horzdist);
vertdist = unique(vertdist);

for i = 1:length(horzdist)
    distance(i) = sqrt((horzdist(i))^2+(vertdist(i))^2);
    if (distance(i) < robot_radius)
      d(m) = distance(i);
      m = m+1;
    end
end

%% define movement
i = 0;
epsilon = 0.50722;
r0 = 0.257366;
for i = 1:length(d)
    a(i) = (r0/d(i))^12;
    b(i) = (r0/d(i))^6;
end
i = 0;

while (i < length(d))
    v(i+1) = (4*epsilon*(a(i+1)-b(i+1))); % Lennard-Jones Model
    i = i+1;
end

for j = 1:4
     if j == 1
        for k = 1:4
            simPlot(k,1) = robot(k,1)+v(j);
            simPlot(k,2) = robot(k,2)+v(j);
        end
     k = k+1;
     else
         for i = 2:4
             for c = 1:4
                simPlot(k,1) = robot(c,1)+v(i);
                simPlot(k,2) = robot(c,2)+v(i);
                k = k+1;
             end
         end
     end
end

%% plot movement
for l = 1:4
    points1 = simPlot(l,:);
    hold on
    xlim([-1 3])
    ylim([-1 3])
    scatter(points1(1),points1(2),150,'red');
end
pause(1)
for l = 5:8
    simPlot(l,:);
    points1 = simPlot(l,:);
    scatter(points1(1),points1(2),150,'green');
end
pause(1)
for l = 9:12
    simPlot(l,:);
    points1 = simPlot(l,:);
    scatter(points1(1),points1(2),150,'blue');
end
pause(1)
for l = 13:16
    simPlot(l,:);
    points1 = simPlot(l,:);
    scatter(points1(1),points1(2),150,'magenta');
end