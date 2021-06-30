
clear
clc
clf

% epsilon = 0.01;
% epsilon_not = 0.01;
% alpha = 2;
% r_not = 4;
% mass = 0.1;
% 
% radius = 0.8:0.1:5;
% chargeProduct = -16:0.5:16;
% 
% for i = 1:length(radius)
%     for j = 1:length(chargeProduct)
%         CBPotential(i,j) = (epsilon .* (6./(alpha - 6) .* exp(alpha)...
%                 * (1 - radius(i)./r_not) - alpha./(alpha - 6) .* (r_not./radius(i)).^6)...
%                 + chargeProduct(j)./(4.*pi.*epsilon_not.*radius(i)));
%     end
% end
%                 
% surf(CBPotential);

r = 0.2;
theta = [0:0.05:2*pi]';
circle = [r*cos(theta),r*sin(theta)];
rect = [0, 0; 0, 1; 1, 1];

[S, D] = MinkSum(circle, rect);
%plot(S(:,1),S(:,2))
pgon = polyshape(S(:,1),S(:,2), 'Simplify', false);
conv = convhull(pgon);
plot(pgon)
hold on
plot(conv);
d = inpolygon(1.4,1.4,conv.Vertices(:,1),conv.Vertices(:,2));

