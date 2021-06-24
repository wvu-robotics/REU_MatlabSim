
clear
clc
clf

epsilon = 0.01;
epsilon_not = 0.01;
alpha = 2;
r_not = 4;
mass = 0.1;

radius = 0.8:0.1:5;
chargeProduct = -16:0.5:16;

for i = 1:length(radius)
    for j = 1:length(chargeProduct)
        CBPotential(i,j) = (epsilon .* (6./(alpha - 6) .* exp(alpha)...
                * (1 - radius(i)./r_not) - alpha./(alpha - 6) .* (r_not./radius(i)).^6)...
                + chargeProduct(j)./(4.*pi.*epsilon_not.*radius(i)));
    end
end
                
surf(CBPotential);








