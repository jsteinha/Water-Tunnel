function [xtape,vtape,utape,ttape] = idtape2(T,omega)
%
dt = 0.0005;
ttape = 0:dt:T-1e-10;
xtape = 0.5*cos(omega*ttape);
vtape = -0.5*omega*sin(omega*ttape);
utape = -0.5*omega^2*cos(omega*ttape);

end