dt = 0.003;
N = 12;
b1 = zeros(N,3);
b2 = zeros(N,3);
for trial = 9:N
    out = dlmread(sprintf('out_sysid%d.dat',trial));
    y = out(1:2:end-1)';
    theta = out(2:2:end-1)';
    ttape = dt * (1:numel(y));
    figure(trial+50)
    plot(ttape,y,ttape,theta)
end
%%
b1
b2
%%
T = zeros(N,1);
for i=1:N
    T(i) = (b2(i,3)-b2(i,2)*1i)/(b1(i,3)-b1(i,2)*1i);
    T(i) = T(i)/(-omegas(i)^2);
end
[omegas' T]
hold off
plot(omegas,abs(T),'b',omegas,phase(T),'r')
%%
p=rationalfit(omegas/(2*pi),T,-10,[],0,0,4);
a=-p.C(1)-p.C(2); b = p.A(1)*p.C(2)+p.A(2)*p.C(1); c = -p.A(1)-p.A(2); d=p.A(1)*p.A(2);
%%
hold on
P = @(s)-(a*s+b)./(s.^2+c*s+d);
T2 = P(1i*omegas);
plot(omegas,abs(T2),'b',omegas,phase(T2),'r')