dt = 0.003;
trials = [1 2 5 15 20 30];
N = numel(trials);
b1 = zeros(N,3);
b2 = zeros(N,3);
b3 = zeros(N,3);
omegas = zeros(N,1);
F = [0.8 -0.277 0.56 -0.42];

for i = 1:N
    trial = trials(i);
    out = dlmread(sprintf('out_sysid%d.dat',trial));
    y = out(1:2:end-1)';
    ttape = dt * (1:numel(y));
    theta = out(2:2:end-1)';
    params = fscanf(fopen(sprintf('misc%d.dat',trial),'r'),'frequency: %22f\nmagnitude:%22f\n');
    omega = params(1)
    mag = params(2)
    in = dlmread(sprintf('sysid%d.dat',trial));
    ytilde = in(2:2:end);
    ydottilde = in(3:2:end);
    u = -(F*[ytilde';zeros(1,numel(ytilde));ydottilde';zeros(1,numel(ydottilde))])';
    A = [ones(numel(ttape),1) sin(omega*ttape)' cos(omega*ttape)'];
    b1(i,:) = A\y;
    b2(i,:) = A\theta;
    b3(i,:) = A\u;
    omegas(i) = params(1);
    ks(i) = round(dt*numel(y)*omegas(i)/(2*pi));
    ks(i)
    uhat = fft(u);
    figure(1);
    subplot(3,2,i);
    hold off
    plot((0:numel(uhat)-1)/(dt*numel(y)),abs(uhat))
    hold on
    stem(ks(i)/(dt*numel(y)),abs(uhat(ks(i)+1)),'r')
    axis([0 2.0 0 max(abs(uhat(2:end)))])
    thetahat = fft(theta);
    figure(2);
    subplot(3,2,i)
    hold off
    plot((0:numel(thetahat)-1)/(dt*numel(y)),abs(thetahat))
    hold on
    stem(ks(i)/(dt*numel(y)),abs(thetahat(ks(i)+1)),'r')
    axis([0 2.0 0 max(abs(thetahat(2:end)))])
    yhat = fft(y);
    figure(3);
    subplot(3,2,i)
    hold off
    plot((0:numel(yhat)-1)/(dt*numel(y)),abs(yhat))
    hold on
    stem(ks(i)/(dt*numel(y)),abs(yhat(ks(i)+1)),'r')
    axis([0 2.0 0 max(abs(yhat(2:end)))])
    format long
    disp(sprintf('magnitude response at omega = %22f',omegas(i)));
    disp(sprintf('least squares:                %22f',sqrt(b2(i,2)^2+b2(i,3)^2)));
%    disp(sprintf('fft:                          %22f',abs(thetahat(15))));
%    x=sqrt(b2(i,2)^2+b2(i,3)^2);
%    y=abs(thetahat(15));
    %figure(i)
    %hold off
    %plot(ttape,y,ttape,A*b1(i,:)')
    %figure(i+N)
    %hold off
    %plot(ttape,theta,ttape,A*b2(i,:)')
    %figure(i+2*N)
    %hold off
    %plot(ttape,u,ttape,A*b3(i,:)')
end
%%
sk = 1i*omegas;
thetak = 0.5*(b2(:,3)-b2(:,2)*1i);
yk = 0.5*(b1(:,3)-b1(:,2)*1i);
uk1 = 0.5*(b3(:,3)-b3(:,2)*1i);
uk2 = sk.^2.*yk-[yk thetak sk.*yk sk.*thetak]*F';
figure(4);
hold off
plot(omegas/(2*pi),abs(uk1),'r.',omegas/(2*pi),abs(uk2),'b.')

%%
b1
b2
%%
T = zeros(N,1);
for i=1:N
    T(i) = (b2(i,3)-b2(i,2)*1i)/2.0; %(b1(i,3)-b1(i,2)*1i);
    %T(i) = T(i)/(1i*omegas(i)).^1;
end
[omegas T]
figure(98)
hold off
plot(omegas,abs(T),'b.'); %,omegas,phase(T),'r.')
%%
alpha = 1.0;
K = 5.0;
sk = 1i*omegas;
thetak = 0.5*(b2(:,3)-b2(:,2)*1i);
yk = 0.5*(b1(:,3)-b1(:,2)*1i);
uk1 = 0.5*(b3(:,3)-b3(:,2)*1i);
uk2 = sk.^2.*yk-[yk thetak sk.*yk sk.*thetak]*F';
figure(96);
plot(omegas,abs(uk1),omegas,abs(uk2),omegas,real(uk1-uk2),omegas,imag(uk1-uk2))
uk = alpha*uk1+(1-alpha)*uk2;
X = [yk thetak sk.*yk sk.*thetak uk]; Y = -sk.^2.*thetak;
X = [X;conj(X)]; Y = [Y;conj(Y)];
X=[X;[K 0 0 0 -K*F(1)]];
Y=[Y;0];
coefs = X\Y;
figure(97);
plot(X*coefs-Y,'.')
cs = coefs;
d = cs(5);
a = cs(2)-F(2)*d;
b = cs(3)-F(3)*d;
c = -cs(4)+F(4)*d;
real([a;b;c;d])
%%
p=rationalfit(omegas/(2*pi),T); %,-10,[],0,0,4);
p.A
p.C
a=-p.C(1)-p.C(2); b = p.A(1)*p.C(2)+p.A(2)*p.C(1); c = -p.A(1)-p.A(2); d=p.A(1)*p.A(2);
%%
figure(98)
hold on
%P = @(s)-(a*s+b)./(s.^2+c*s+d);
%omegas2 = omegas;%(0:.1:100);
%T2 = P(1i*omegas2);%omegas);
T2 = (cs(1)*yk+cs(3)*sk.*yk+cs(5).*uk)./(sk.^2-cs(4)*sk-cs(2));
plot(omegas,abs(T2),'b'); %,omegas,phase(T2),'r')