function [d]=delta(x,y,h)

% This computes the delta function's output given:
%
% (x,y): input of the form (x-X,y-Y)
% h: delta function parameter
% (X,Y): point around which delta function is defined
%
%
%PRELIMINARY STEP 3: Define an approximate delta function
%
%delta(x)=1/epsilon*phi(x/epsilon), r=x/epsilon
%3/8-|r|/4+1/8*sqrt(1+4|r|-4r^2)   0.leq.|r|.leq.1
%5/8-|r|/4-1/8*sqrt(-7+12|r|-4r^2) 1.leq.|r|.leq.2
%0 |r|>2
rx=abs(x)/h;
ry=abs(y)/h;

if (rx <= 1)
    dx=3/8-abs(rx)/4+1/8*sqrt(1+4*abs(rx)-4*rx^2);
elseif (rx <= 2)
    dx=5/8-abs(rx)/4-1/8*sqrt(-7+12*abs(rx)-4*rx^2);
else
    dx=0;
end

if (ry <= 1)
    dy=3/8-abs(ry)/4+1/8*sqrt(1+4*abs(ry)-4*ry^2);
elseif (ry <= 2)
    dy=5/8-abs(ry)/4-1/8*sqrt(-7+12*abs(ry)-4*ry^2);
else
    dy=0;
end

d=1/h*dx*1/h*dy;

% Note: This function is extremely well approximated by .25(1+cos(pi*r/2))