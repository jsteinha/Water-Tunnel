function [theta_out,theta]=PDEFlap(theta_in,yp_0,t_end, run);

str=sprintf('Starting run %d...',run);
disp(str);


%mex singleton.f90 IBPlateMex.F90 IBPlateMexGateway.F90 % Only need to compile here if you change IBPlateMex.f90
%mex IBPlateMex.F90 singleton.f90 IBPlateMexGateway.F90
% ODEFlap
%
%  INPUT: Orientation angle (in rad), and final
%  time (in s).
%
%  OUTPUT: Angle at t=t_end and for all t.
%
dt=0.0005; %This must match that defined in IBPlateMex.f90; I suggest leaving alone
T=t_end/dt;

movie = 0; %1 to make a movie, 0 if not [movies take longer due to data printouts etc.]
str = sprintf('rm -f *%d.txt',run);
system(str);

Poften = round(T/100); % Get 100 timesteps for the movie

display('Entering IBPlateMex...');
theta=IBPlateMex(theta_in,yp_0,T,movie,Poften,run);
theta_out=theta(end);

thetadot = zeros(T,1);
for time=1:T
    if (time==1)
        thetadot(time)=(theta(time)-theta_in)/dt;  %thetadot
    else
        thetadot(time)=(theta(time)-theta(time-1))/dt;  %thetadot
    end
end

return

