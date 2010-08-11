k1 = 10; k2 = 279; k3 = 130; k4 = 15.3;
%k1 = 0; k2 = 0; k3 = 0; k4 = 0;
c1 = 1; c2 = 1; M = 20; a = 10;
f = [k1 -k2 k3 -k4];
f = [0 0 0 0];
A = [[0 0 1 0];[0 0 0 1];f;[0 c1 c2 -M]+a*f];
b = [0;0;1;a];
sys = ss(A,b,eye(4),[])
[V,D]=eig(A)
lqr(sys,diag([10 10 1 1]),.1)
%%
K=2; T1 = 100; T2 = 100;
dt = 0.003;
starts = [T1/3 2*T1/3];%sort(T1*rand(K,1));
widths = ones(K,1); %rand(K,1);
heights = ones(K,1); %randn(K,1);
ttape = 0:dt:T2;
utape = zeros(size(ttape));
for i=1:K
    st = 1+round(starts(i)/dt);
    en = round((starts(i)+widths(i))/dt);
    en = min(length(utape),en);
    if st <= en
        utape(st:en) = utape(st:en) + heights(i) * ones(size(st:en));
    end
end
%utape = ones(size(ttape));
%
sfigure(3);
plot(ttape,utape)
%
sfigure(1); clf;
y=lsim(sys,utape,ttape);
%plot(y(:,1),y(:,2),'.')
%hold on
plotyy(ttape,y(:,2),ttape,y(:,1))
%axis([0 T2/dt -2 2])
%%
x = y(:,1);
xdot = y(:,2);
xddot = ([xdot(2:end);0]-[0;xdot(1:end-1)])/(2*dt);
xddot(1) = 0; xddot(end) = 0;
%%
coefs = [x xdot]\xddot
coefs2 = x\xdot
