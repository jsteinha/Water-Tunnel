dt = 0.003;
trials = [1 2 5 15 20 30];
N = numel(trials);
b10 = zeros(N,1);
b20 = zeros(N,1);
b1 = zeros(N,1);
b2 = zeros(N,1);
b3 = zeros(N,1);
omegas = zeros(N,1);
F = [0.8 -0.277 0.56 -0.42];

for i = 1:N
    trial = trials(i);
    out = dlmread(sprintf('out_sysid%d.dat',trial));
    y = out(1:2:end-1)';
    theta = out(2:2:end-1)';
    params = fscanf(fopen(sprintf('misc%d.dat',trial),'r'),'frequency: %22f\nmagnitude:%22f\n');
    in = dlmread(sprintf('sysid%d.dat',trial));
    ytilde = in(2:2:end);
    ydottilde = in(3:2:end);
    u = -(F*[ytilde';zeros(1,numel(ytilde));ydottilde';zeros(1,numel(ydottilde))])';
    y = y(1:numel(u));
    theta = theta(1:numel(u));
    ttape = dt * (0:numel(y)-1);
    omegas(i) = params(1);
    ks(i) = round(dt*numel(y)*omegas(i)/(2*pi));

    stemx = ks(i)/(dt*numel(y));
    xrange = (0:numel(y)-1)/(dt*numel(y));
    
    uhat = fft(u);
    figure(1);
    subplot(3,2,i);
    hold off
    plot(xrange,real(uhat),'b',xrange,imag(uhat),'g')
    hold on
    ub = 0.5*(uhat(ks(i))+uhat(ks(i)+2));
    un = uhat(ks(i)+1);
    plot([stemx stemx],[real(ub) real(un)],'ro-')
    plot([stemx stemx],[imag(ub) imag(un)],'yo-')
    axis([0 2.0 -max(abs(uhat(2:end))) max(abs(uhat(2:end)))])
    
    thetahat = fft(theta);
    figure(2);
    subplot(3,2,i)
    hold off
    plot(xrange,real(thetahat),'b',xrange,imag(thetahat),'g')
    hold on
    tb = 0.5*(thetahat(ks(i))+thetahat(ks(i)+2));
    tn = thetahat(ks(i)+1);
    plot([stemx stemx],[real(tb) real(tn)],'ro-')
    plot([stemx stemx],[imag(tb) imag(tn)],'yo-')
    axis([0 2.0 -max(abs(thetahat(2:end))) max(abs(thetahat(2:end)))])
    
    yhat = fft(y);
    figure(3);
    subplot(3,2,i)
    hold off
    plot(xrange,real(yhat),'b',xrange,imag(yhat),'g')
    hold on
    yb = 0.5*(yhat(ks(i))+yhat(ks(i)+2));
    yn = yhat(ks(i)+1);
    plot([stemx stemx],[real(yb) real(yn)],'ro-')
    plot([stemx stemx],[imag(yb) imag(yn)],'yo-')
    axis([0 2.0 -max(abs(yhat(2:end))) max(abs(yhat(2:end)))])
    
    b10(i) = yhat(ks(i)+1);
    b20(i) = thetahat(ks(i)+1);
    b1(i) = yhat(ks(i)+1)-yb;
    b2(i) = thetahat(ks(i)+1)-tb;
    b3(i) = uhat(ks(i)+1)-ub;
end

%%
sk = 1i*omegas;
thetak = b2;
yk = b1;
uk1 = b3;
uk2 = sk.^2.*yk-[yk thetak sk.*yk sk.*thetak]*F';
thetak0 = b20;
yk0 = b10;
uk20 = sk.^2.*yk0-[yk0 thetak0 sk.*yk0 sk.*thetak0]*F';
figure(4);
hold off
plot(omegas/(2*pi),abs(uk1-uk2)./abs(uk1),'r.',omegas/(2*pi),abs(uk1-uk20)./abs(uk1),'b.'); %,omegas/(2*pi),abs(uk20),'g.')
axis([0 1.0 0 1.0])