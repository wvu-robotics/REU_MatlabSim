close all 
clear
clc

%% instantiate variables and robots
i = 0;
k = 5;
robot = zeros(5,2);

while (i < k)
    robot(i+1,1) = rand(1),rand(1);
    robot(i+1,2) = rand(1),rand(1);
    i = i+1;
end

%% define movement
i = 0;
t = linspace(0,1,5);

while (i < length(t))
        v(i+1) = 1.075*t(i+1); % force-based model
        i = i+1;
end

for j = 1:5
        if j == 1
            for k = 1:5
                simPlot(k,1) = robot(k,1)*v(k);
                simPlot(k,2) = robot(k,2)*v(k);
            end
        else
            for k = 6:25
                simPlot(k,1) = simPlot(k-1,1)*v(j) + v(j-1);
                simPlot(k,2) = simPlot(k-1,2)*v(j) + v(j-1);
            end
        end
end

%% plot movement
for l = 1:5
    points1 = simPlot(l,:);
    hold on
    xlim([-1 50])
    ylim([-1 50])
    scatter(points1(1),points1(2),150,'red');
end
pause(1)
for l = 6:10
    simPlot(l,:);
    points1 = simPlot(l,:);
    scatter(points1(1),points1(2),150,'green');
end
pause(1)
for l = 11:15
    simPlot(l,:);
    points1 = simPlot(l,:);
    scatter(points1(1),points1(2),150,'blue');
end
pause(1)
for l = 16:20
    simPlot(l,:);
    points1 = simPlot(l,:);
    scatter(points1(1),points1(2),150,'magenta');
end
pause(1)
for l = 21:25
    simPlot(l,:);
    points1 = simPlot(l,:);
    scatter(points1(1),points1(2),150,'black');
end