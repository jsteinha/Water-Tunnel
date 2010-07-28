
dt = 0.003;
omegas = 4*pi; %linspace(1.0,5.0,30);
mags = 0.03; %0.03./omegas;
Tgoal = 60.0;

for i=1:numel(omegas)
    k = round(omegas(i)*Tgoal/(2*pi));
    T = k*(2*pi/omegas(i));
    T = dt*round(T/dt);
    omegas(i) = 2*pi*k/T;
    ttape = 0:dt:T-1e-6;
    yin = mags(i)*sin(omegas(i)*ttape);
    ydotin = mags(i)*omegas(i)*cos(omegas(i)*ttape);
    yhat = fft(yin);
    k+1
    numel(ttape)-k+1
    for j=1:numel(yhat)
        if(abs(yhat(j)) > 1e-9)
            disp(sprintf('yhat(%d,%d) = %22f',i,j,abs(yhat(j))));
        end
    end
    dlmwrite(sprintf('sysid%d.dat',32+i),[numel(yin) reshape([yin;ydotin],[1 numel(yin)+numel(ydotin)])]');
    fprintf(fopen(sprintf('misc%d.dat',32+i),'w'),'frequency: %22f\nmagnitude:%22f\n',omegas(i),mags(i));
end

