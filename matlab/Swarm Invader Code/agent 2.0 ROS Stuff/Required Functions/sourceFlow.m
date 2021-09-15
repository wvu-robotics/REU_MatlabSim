function [Vxi,Vyi,Psi,Phi] = sourceFlow(xi,yi,S,X,Y)
%SourceFlow Calculates the x and y component of the path vector due to a
%source flow
%   Uses Laplacian Transforms to calculate x and y components from global
%   corrdinates X,Y, the source location xi,yi, and strength S

r = sqrt((X-xi).^2+(Y-yi).^2);
    theta = atan2((Y-yi),(X-xi));
    if theta < 0
        theta = theta +2*pi;
    end
    Vr = S./(2.*pi.*r);
    Vxi = Vr.*cos(theta);
    Vyi = Vr.*sin(theta);
    Psi = theta.*S./(2.*pi);
    Phi = S./(2.*pi).*log(r);

end

% This code was created by Trevor Smith, used in WVU NSF REU Summer 2021