dt = 0.003;
N = 12;
b1 = zeros(N,3);
b2 = zeros(N,3);
for trial = 1:N
    out = dlmread(sprintf('out_sysid%d.dat',trial));
    ttape = dt * (1:numel(ydot));
    y = out(1:2:end-1)';
    theta = out(2:2:end-1)';
    params = fscanf(fopen(sprintf('misc%d.dat',trial),'r'),'frequency: %22f\nmagnitude:%22f\n');
    omega = params(1)
    mag = params(2)
    A = [ones(numel(ttape),1) sin(omega*ttape)' cos(omega*ttape)'];
    b1(trial,:) = A\y;
    b2(trial,:) = A\theta;
end
%%
b1
b2
%%
T = zeros(N,1);
for i=1:N
    T(i) = (b2(i,3)-b2(i,2)*1i)/(b1(i,3)-b1(i,2)*1i);
    T(i) = T(i)/(1i*omegas(i));
end
[omegas' T]
hold off
plot(omegas,abs(T),'b',omegas,phase(T),'r')
%%
p=rationalfit(omegas/(2*pi),T);
a=-p.C(1)-p.C(2); b = p.A(1)*p.C(2)+p.A(2)*p.C(1); c = -p.A(1)-p.A(2); d=p.A(1)*p.A(2);
%%
hold on
P = @(s)-(a*s+b)./(s.^2+c*s+d);
T2 = P(1i*omegas);
plot(omegas,abs(T2),'b',omegas,phase(T2),'r')