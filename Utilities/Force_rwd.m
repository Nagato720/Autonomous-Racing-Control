%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAE 502 Vehicle Control Systems Project 
% Formula 1 car racing: subfunctions
%   Copyright (C)2025 Chaozhe He. All Rights Reserved.
%   Author: Prof. Chaozhe He
%           Department of Mechanical and Aerospace Engineering
%           SUNY University at Buffalo
%           March 2025
% Any issues/bug reports,
% please email to chaozheh@buffalo.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gives the lateral forces model for rear wheel drive model
function [Ffl,Frl]=Force_rwd(sigma,R,gamma,gammadot,m,m0,d,L)
    Frl= -m/L*(1-d/L)*tan(gamma).*sigma.^2 ...
          +1/L./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2)...
        .*(m*(m0*L-m*d)*gammadot.*sigma ...
          +(m0*L-m*d)*R.*sin(gamma).*cos(gamma)) ;
      
    Ffl= -m*d/L^2*tan(gamma)./cos(gamma).*sigma.^2 ...
            -m*m0*gammadot.*sigma./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2)./(cos(gamma)) ...
            -m0*R.*sin(gamma)./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2);
end
