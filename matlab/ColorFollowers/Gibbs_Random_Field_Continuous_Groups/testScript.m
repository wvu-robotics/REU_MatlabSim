
% 
% radius = 2.0:0.01:5;
% epsilon = 0.04;
% epsilon_not = 0.04;
% alpha = 1;
% r_not = 8;
% mass = 0.1;
%         
% chargeProduct = -16;
%         
% CBPotential = (epsilon .* (6./(alpha - 6) .* exp(alpha)...
%                     * (1 - radius./r_not) - alpha./(alpha - 6) .* (r_not./radius).^6)...
%                     + chargeProduct./(4.*pi.*epsilon_not.*radius));
%                 
% plot(radius, CBPotential);


RGB = [1 0 0];

x = 1:0.1:10;
y = 3.*x.^2;

plot(x,y,'.','MarkerEdge', RGB);






