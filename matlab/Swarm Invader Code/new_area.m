%Robot Team Spawn is random
rng('default');
x = rand(15,1);
y = rand(15,1);
plot(x,y,'.')
xlim([0 10])
ylim([0 10])
hold on

OBSTACLES = 1;
ROBOTS = 2;
INVADERS = 3;
map = zeros(10,10,3);
map(3:5,4:10,OBSTACLES)=1;
%Invader Team Spawn is random
% a = rand(1,1);
% b = rand(4,4);
% plot(a,b,'x')
% hold on
% 
% %Goal Spawn is fixed
% c = 0.5;
% d = 0.5;
% plot(c,d,'*')
% 
% %Boundary of Game
% j = boundary(x,y,0.1);
% hold on;
% plot(x(j),y(j));
% 
% n = 20; %How many steps
% xy = 1 * rand(2,n)+1; %Next point, right now completely random



