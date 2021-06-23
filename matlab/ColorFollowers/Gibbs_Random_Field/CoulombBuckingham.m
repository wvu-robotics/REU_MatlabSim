function CBPotential = CoulombBuckingham(centralPosition, neighborPosition, centralClass, neighborClass, centralCharge, neighborCharge, epsilon, epsilon_not, alpha, r_not)
    radius = norm(centralPosition - neighborPosition);
    
    if centralClass == neighborClass
        chargeProduct = abs(centralCharge*neighborCharge);
    else
        chargeProduct = -abs(centralCharge*neighborCharge);
    end
    
    
    CBPotential = epsilon * (6/(alpha - 6) * exp(alpha) * (1 - radius/r_not) - alpha/(alpha - 6) * (r_not/radius)^6) + chargeProduct/(4*pi*epsilon_not*radius);

end