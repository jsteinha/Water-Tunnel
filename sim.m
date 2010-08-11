M = 40.0; c = -2.0;
A = [[0 1];[c -M]];
b = [0;1];
sys = ss(A,b,eye(2),[])
%%
K=2; T1 = 300; T2 = 300;
dt = 0.003;
starts = [T1/3 2*T1/3]; %sort(T1*rand(K,1));
widths = 25*ones(K,1);%rand(K,1);
heights = 1.0*ones(K,1); %randn(K,1);
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
%%
sfigure(3);
plot(ttape,utape)
%%
sfigure(1);
y=lsim(sys,utape,ttape);
%plot(y(:,1),y(:,2),'.')
hold on
plot(-1.0*y(:,1)+20*y(:,2),'g.')
axis([0 T2/dt -2 2])
%%
x = y(:,1);
xdot = y(:,2);
xddot = ([xdot(2:end);0]-[0;xdot(1:end-1)])/(2*dt);
xddot(1) = 0; xddot(end) = 0;
%%
coefs = [x xdot]\xddot
coefs2 = x\xdot
%%
tau = 0.001;
Nu = 20; N = length(x)
plotting=0; saving=1;
if plotting
    sfigure(2); clf; hold on;
end
tl = 1;
tr = 1;
lastfailure = 0;
intervals = zeros(0,2);

for i=1:N
    while norm([x(i)-x(tl),xdot(i)-xdot(tl)],2) >= tau
        tl = tl+1;
    end
    while tr <= length(x) && norm([x(i)-x(tr),xdot(i)-xdot(tr)],2) < tau
        tr = tr+1;
    end
    if tr-tl >= Nu
        if plotting
            plot(x(i),xdot(i),'r.');
        end
    else
        if saving
           if lastfailure < i-1
               intervals = [intervals;[lastfailure+1 i-1]];
           end
           lastfailure = i;
        end
    end
end
if saving
    intervals
end
if plotting
    drawnow;
    hold off;
end
%%
intervals = [
    %[1 19977];
    %[20034+80 20276-20];
    [20331+150 23165-20];
    %[23198+50 23383-20]
    ];
ptape = [];
for i=1:size(intervals,1)
    ptape = [ptape intervals(i,1):intervals(i,2)];
end
sfigure(2);
plot(x(ptape),xdot(ptape),'r.')
%%
R = sum(intervals(:,2)-intervals(:,1)+1);
C = size(intervals,1)+1;
As = zeros(R,C);
bs = zeros(R,1);
cnt = 0;
for i=1:C-1
    r1 = cnt+1;
    r2 = cnt+intervals(i,2)-intervals(i,1)+1;
    cnt = cnt+intervals(i,2)-intervals(i,1);
    As(r1:r2,1) = x(intervals(i,1):intervals(i,2));
    As(r1:r2,i+1) = ones(r2-r1+1,1);
    bs(r1:r2) = xdot(intervals(i,1):intervals(i,2));
end
As\bs
%%
coefs = [x(indices) xdot(indices) ones(size(indices))]\xddot(indices)
coefs2 = x(indices)\xdot(indices)
%%
m=n4sid(iddata([x xdot]),2)
%%
m=arx(iddata(xdot),2,'focus','simulation')
