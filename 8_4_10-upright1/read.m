K=4;
%%
data = cell(K,1);
for i=1:K
    data{i} = dlmread(sprintf('out_sysid1_gains%d.dat',i));
    data{i} = reshape(data{i},[numel(data{i}) 1]);
end
%%
y = cell(K,1);
theta = cell(K,1);
for i=1:K
    y{i} = data{i}(1:2:end-1);
    theta{i} = data{i}(2:2:end);
end
%%
dt = 0.003;
N = cell(4,1);
T = cell(4,1);
ttape = cell(4,1);
for i=1:4
    N{i} = numel(theta{i});
    T{i} = dt*N{i};
    ttape{i} = dt*(0:N{i}-1)';
end
%%
sfigure(1);
clf;
hold on;
for i=1:K
    plot(ttape{i},y{i});
end
sfigure(2);
clf;
hold on;
for i=1:K
    plot(ttape{i},theta{i});
end
%%
K = 5;
As = 8000;
Ns = 15000;
tol = 1e-9;
sigma = 0.5;
mu = 0.7;
wt = @(t)exp(-(t-mu).^2/sigma^2);
wts = wt(2*ttape(1:Ns));
Stape = zeros(Ns,1);
figure(1);
subplot(2,1,2);
plot(ttape(1:Ns),y(1:Ns));

for i=As:1:Ns
    figure(1);
    subplot(2,1,1);
    vals = wts.*diffs(y(1:Ns),i,dt)';
    Stape(i) = sum(vals);
    plot(2*ttape(1:Ns),vals);
    axis([0 20.0 -1e-1 1e-1]);
    drawnow;
    figure(1);
    subplot(2,1,2);
    hold off;
    plot(ttape(1:Ns),y(1:Ns),'b');
    hold on;
    plot(ttape(1:i),dt*Stape(1:i),'g');
    plot([ttape(i) ttape(i)],[-0.05 0.05],'r');
    axis([ttape(i)-5.0,ttape(i)+5.0 -0.05 0.05]);
    drawnow;
%    pause(2.0);
    i
end
%%
figure(5);
plot(ttape(1:Ns),y(1:Ns),ttape(1:Ns),Stape);
%%
figure(4)
plot(isMin)
%%
yhat = fft(y);
thetahat = fft(theta);
wtape = (1/T)*(0:N-1);
figure(3);
plot(wtape,real(yhat),wtape,imag(yhat));
%axis([0 2.0 min([real(yhat(2:end)) imag(yhat(2:end))]) max([real(yhat(2:end)) imag(yhat(2:end))])])
axis([0 2.0 -300.0 300.0])