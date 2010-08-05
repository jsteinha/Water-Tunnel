%% match filter
ys = smoothing(y);
dy = ([ys(2:end);0]-[0;ys(1:end-1)])/(2*dt);
d2y = ([dy(2:end);0]-[0;dy(1:end-1)])/(2*dt);
thetas = smoothing(theta);
dtheta = ([thetas(2:end);0]-[0;thetas(1:end-1)])/(2*dt);
d2theta = ([dtheta(2:end);0]-[0;dtheta(1:end-1)])/(2*dt);
%%
start=10000;
fin=60000;
t2=ttape(start:fin);
th2=thetas(start:fin);
y2 = ys(start:fin);
dy2 = dy(start:fin);
%%
N=numel(t2);
match=zeros(size(t2));
sigma = 0.4;
for i=1:N
    match(i) = sum(exp(-(t2-t2(i)).^2/sigma^2).*dy2);
end
%%
figure(1)
hold off
alpha = 0.00425;
plot(t2,dy2,t2,alpha*match,t2,th2,'k')
%%
figure(1)
hold on
z2 = y(start:fin);
consolidate = zeros(size(t2));
cy = zeros(size(t2));
dat = [];
for i=2:N-1
    if alpha*match(i) > 0.005 && match(i) > match(i-1) && match(i) > match(i+1)
        plot(t2(i),alpha*match(i),'ro');
        if numel(dat) == 0
            dat = iddata(y2(i-30:i+30),z2(i-30:i+30),dt);
        else
            dat = merge(dat,iddata(y2(i-30:i+30),z2(i-30:i+30),dt));
        end
        consolidate = consolidate + circshift(y2,round(numel(t2)/2)-i);
        cy = cy + circshift(z2,round(numel(t2)/2)-i);
    end
    if alpha*match(i) < -0.005 && match(i) < match(i-1) && match(i) < match(i+1)
        plot(t2(i),alpha*match(i),'ro');
        if numel(dat) == 0
            dat = iddata(y2(i-30:i+30),z2(i-30:i+30),dt);
        else
            dat = merge(dat,iddata(y2(i-30:i+30),z2(i-30:i+30),dt));
        end
        consolidate = consolidate - circshift(y2,round(numel(t2)/2)-i);
        cy = cy - circshift(z2,round(numel(t2)/2)-i);
    end
end

figure(2);
st2 = round(numel(t2)/2-5.0/dt);
fi2 = round(numel(t2)/2+5.0/dt);
%%
plot(t2(st2:fi2),consolidate(st2:fi2),t2(st2:fi2),10*cy(st2:fi2));

%%
figure(2);
sta = round(60/dt-start);
fia = round(140/dt-start);
at = t2(sta:fia);
ac = consolidate(sta:fia);
acy = cy(sta:fia);
plot(at,ac,at,10*acy);

%%
dy = df(y')/dt;
dz2 = dy(start:fin);
plot(t2,dz2,t2,y2)


%%
A = [];
b = [];
for i=1:132
    i
    e = getexp(dat,i);
    th = e.y;
    yi = e.u;
    A = [A;[th(2:end-1) th(1:end-2) (0.5/dt)*(yi(3:end)-yi(1:end-2)) (1/dt^2)*(yi(3:end)-2*yi(2:end-1)+yi(1:end-2))]];
    b = [b;th(3:end)-th(2:end-1)];
end
%%
coef = A\b