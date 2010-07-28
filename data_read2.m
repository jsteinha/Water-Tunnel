dt = 0.003;
trials = 1:24;
N = numel(trials);
b1 = zeros(N,3);
b2 = zeros(N,3);
omegas = zeros(N,1);
for i = 1:N
    trial = trials(i);
    out = dlmread(sprintf('out_sysid%d.dat',trial));
    y = out(1:3:end-1)';
    ttape = dt * (1:numel(y));
    theta = out(2:3:end-1)';
    params = fscanf(fopen(sprintf('misc%d.dat',trial),'r'),'frequency: %22f\nmagnitude:%22f\n');
    omega = params(1)
    mag = params(2)
    A = [ones(numel(ttape),1) sin(omega*ttape)' cos(omega*ttape)'];
    b1(i,:) = A\y;
    b2(i,:) = A\theta;
    omegas(i) = params(1);
%    thetahat = fft(theta);
    format long
    disp(sprintf('magnitude response at omega = %22f',omegas(i)));
    disp(sprintf('least squares:                %22f',sqrt(b2(i,2)^2+b2(i,3)^2)));
%    disp(sprintf('fft:                          %22f',abs(thetahat(15))));
%    figure(i)
%    x=sqrt(b2(i,2)^2+b2(i,3)^2);
%    y=abs(thetahat(15));
%    hold off
    plot(ttape,y,ttape,theta)
%    hold on
%    plot(ttape,A*b1(i,:)',ttape,A*b2(i,:)')
end
%%
b1
b2
%%
T = zeros(N,1);
for i=1:N
    T(i) = (b2(i,3)-b2(i,2)*1i)/(b1(i,3)-b1(i,2)*1i);
    T(i) = T(i)/(1i*omegas(i)).^1;
end
[omegas T]
hold off
plot(omegas,abs(T),'b',omegas,phase(T),'r')
%%
p=rationalfit(omegas/(2*pi),T); %,-10,[],0,0,4);
p.A
p.C
a=-p.C(1)-p.C(2); b = p.A(1)*p.C(2)+p.A(2)*p.C(1); c = -p.A(1)-p.A(2); d=p.A(1)*p.A(2);
%%
hold on
P = @(s)-(a*s+b)./(s.^2+c*s+d);
T2 = P(1i*omegas);
plot(omegas,abs(T2),'b',omegas,phase(T2),'r')