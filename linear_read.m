dt = 0.003;
trials = 1:24;
N = numel(trials);
b1 = zeros(N,2);
b2 = zeros(N,2);
omegas = zeros(N,1);
ks = zeros(N,1);
plotting = 0;
for i = 1:N
    trial = trials(i);
    out = dlmread(sprintf('out_sysid%d.dat',trial));
    y = out(1:2:end-1)';
    theta = out(2:2:end-1)';
    theta = mod(theta,2*pi)-pi;
    ttape = dt * (1:numel(y));
    params = fscanf(fopen(sprintf('misc%d.dat',trial),'r'),'frequency: %22f\nmagnitude:%22f\n');
    omegas(i) = params(1)
    ks(i) = round(dt*numel(y)*omegas(i)/(2*pi));
    ks(i)
    yhat = fft(y);
    thetahat = fft(theta);
    b1(i,1) =     yhat(1);  b1(i,2) =     yhat(ks(i)+1);
    b2(i,1) = thetahat(1);  b2(i,2) = thetahat(ks(i)+1);
    if plotting
        figure(i)
        hold off
        plot(ttape,y,ttape,theta)
    end
    yfit     = b1(i,1)+2*real(b1(i,2))*cos(omegas(i)*ttape)-2*imag(b1(i,2))*sin(omegas(i)*ttape);
    thetafit = b2(i,1)+2*real(b2(i,2))*cos(omegas(i)*ttape)-2*imag(b2(i,2))*sin(omegas(i)*ttape);
    yfit = yfit / numel(ttape);
    thetafit = thetafit / numel(ttape);
    if plotting
        hold on
        plot(ttape,yfit,ttape,thetafit)
        figure(i+N)
        yh=yhat;%(1:2*ks(i)+1);
        plot([abs(yh)])
        figure(i+2*N)
        thh=thetahat;%(1:2*ks(i)+1);
        plot([abs(thh)])
    end
end
%%
b1
b2
%%
T = zeros(N,1);
for i=1:N
    T(i) = b2(i,2)/b1(i,2);
    T(i) = T(i)/(1i*omegas(i)).^1;
end
[omegas T]
hold off
plot(omegas,abs(T),'b.',omegas,phase(T),'r.')
%%
yk = b1(:,2); sk = (1i*omegas); thetak = b2(:,2); v = 0.2; l = 0.0316;
yk=[yk;conj(yk)]; sk=[sk;conj(sk)]; thetak = [thetak;conj(thetak)];
%A = [ yk.*sk+thetak.*sk*l+thetak*v yk.*sk.^2];
%B = -thetak.*sk.^2;
A = [yk.*sk.^2];
c1v=900;
B = -thetak.*sk.^2-c1v*(yk.*sk+thetak.*sk*l+thetak*v);
coefs = A\B;
c2 = coefs(1)
%%
a=c2; b = c1v; c = c1v*l; d = c1v*v;
%a=coefs(1); b=coefs(2); c=coefs(3); d=coefs(4);
%%
p=rationalfit(omegas/(2*pi),T); %,-10,[],0,0,4);
p.A
p.C
a=-p.C(1)-p.C(2); b = p.A(1)*p.C(2)+p.A(2)*p.C(1); c = -p.A(1)-p.A(2); d=p.A(1)*p.A(2);
%%
hold on
P = @(s)-(a*s+b)./(s.^2+c*s+d);
T2 = P(1i*omegas);
plot(omegas,abs(T2),'b',omegas,phase(T2)-2*pi,'r')