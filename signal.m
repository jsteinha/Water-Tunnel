
T = 60.0;
dt = 0.003;
ttape = 0:dt:T-1e-10;

omegas = linspace(0.1,5.0,30);
mags = 0.03*ones(size(omegas));

for i=1:numel(omegas)
    yin = mags(i)*sin(omegas(i)*ttape);
    dlmwrite(sprintf('sysid%d.dat',i),[numel(yin) yin]');
    fprintf(fopen(sprintf('misc%d.dat',i),'w'),'frequency: %22f\nmagnitude:%22f\n',omegas(i),mags(i));
%    yout = mags(i)/omegas(i)*(1-cos(omegas(i)*ttape));
%    thetaout = 2*mags(i)*sin(omegas(i)*ttape)+0.05*sin((12/7)*omegas(i)*ttape);
%    dlmwrite(sprintf('out_sysid%d.dat',i),[reshape([yout;thetaout],[1 numel(yout)+numel(thetaout)]) 0]);
%    figure(i)
%    plot(ttape,ydot)
end

%%
y = [1 2 3 4]
theta = [5 6 7 8]
out = reshape([y;theta],[1 8])