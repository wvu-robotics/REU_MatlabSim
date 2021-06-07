function Y = flockSetY(L,B);
% input X1Y1, X2Y2
% simple line equation coef
% output Y.A*x + Y.B*y + Y.C = 0

% From parctical reason, let trap division by zero
% and set a close approx of the needed line
if (B(1)-L(1))
    % (Y2-Y1)/(X2-X1)
    Y.A = (B(2)-L(2))/(B(1)-L(1));
else
    Y.A = -1e6*sign(B(2)-L(2))*sign(B(1)-L(1));
%     Y.A = 0; 
end

Y.B = -1;
%     Y1-(Y2-Y1)/(X2-X1)*X(1)
Y.C = L(2)-Y.A*L(1);

return
