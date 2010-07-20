omegas = linspace(0.8,3.0,12);
mags = 0.02*ones(size(omegas));

T = 20.0;
dt = 0.003;
ttape = 0:dt:T;

for i=1:numel(omegas)
    ydot = mags(i)*sin(omegas(i)*ttape);
    dlmwrite(sprintf('sysid%d.dat',i),[numel(ydot) ydot]');
    fprintf(fopen(sprintf('misc%d.dat',i),'w'),'frequency: %22f\nmagnitude:%22f\n',omegas(i),mags(i));
    figure(i)
    plot(ttape,ydot)
end
