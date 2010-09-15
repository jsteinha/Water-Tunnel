%% match filter
ys = cell(K,1);
dy = cell(K,1);
d2y = cell(K,1);
thetas = cell(K,1);
dtheta = cell(K,1);
d2theta = cell(K,1);
for i=1:K
    ys{i} = smoothing(y{i});
    dy{i} = ([ys{i}(2:end);0]-[0;ys{i}(1:end-1)])/(2*dt);
    d2y{i} = ([dy{i}(2:end);0]-[0;dy{i}(1:end-1)])/(2*dt);
    thetas{i} = smoothing(theta{i});
    dtheta{i} = ([thetas{i}(2:end);0]-[0;thetas{i}(1:end-1)])/(2*dt);
    d2theta{i} = ([dtheta{i}(2:end);0]-[0;dtheta{i}(1:end-1)])/(2*dt);
end
%% visualize a trial
figure(1);
trial=4;
plot(ttape{trial},theta{trial})
%axis([20 35 -0.025 0.00])
% plot(fftshift(abs(fft(thetas{trial}))))
% plot(unwrap(angle(hilbert(thetas{trial}(:)))))
% H = unwrap(angle(hilbert(thetas{trial}(:))));
% ((H(end) - H(1))/length(H))
% phi = (H - (1:length(H))'*((H(end)-H(1))/length(H)));
% plot(phi)
% 
% plot(smoothing(phi))


%%
start=[1.3 5.0 0.42 5.0];
fin=[120.0 102.0 12.0 165];
si=round(start/dt);
ei=round(fin/dt);
t2=cell(K,1); th2=cell(K,1); dth2=cell(K,1); d2th2=cell(K,1);
y2=cell(K,1); dy2=cell(K,1); d2y2=cell(K,1);
for i=1:K
    region = si(i):ei(i);
    t2{i}=ttape{i}(region);
    th2{i}=thetas{i}(region);
    dth2{i}=dtheta{i}(region);
    d2th2{i}=d2theta{i}(region);
    y2{i}=ys{i}(region);
    dy2{i}=dy{i}(region);
    d2y2{i}=d2y{i}(region);
    N{i}=numel(region);
end
clear region;
%%

%% break the data up into regions, discard non-linear regions
ts = 0.19; % timescale for a region
alpha = 1.09; % only take regions where norm(Ax-b,2) < alpha * norm(b,2)

A = [];
b = [];
plotting = 0;
for i=1:K
    d = round(ts/dt);
    for j=1:N{i}/d
        ptape = d*(j-1)+1:d*j;
        tt = dt*(0:numel(ptape)-1)';
        nA = [th2{i}(ptape) dth2{i}(ptape) y2{i}(ptape) dy2{i}(ptape) d2y2{i}(ptape) ones(numel(ptape),1)];
        nb = [d2th2{i}(ptape)];
        nc = nA\nb;
        if norm(nA*nc-nb,2) < alpha * norm(nb,2)
            if plotting
                sfigure(4);
                clf;
                plot(tt,d2th2{i}(ptape),'r',tt,nA*nc,'g'); %,tt,nA*coefs,'b');
                axis([min(tt) max(tt) -20 20]);
                sfigure(6);
                clf;
                plot(tt,th2{i}(ptape)/max(th2{i}));
                sfigure(7);
                clf;
                plot(tt,dy2{i}(ptape)/max(dy2{i}));
                sfigure(8);
                clf;
                plot(tt,dth2{i}(ptape),tt,dth3{i}(ptape));
                i
                pause(4.0);
            end
            if numel(A) == 0
                A = nA;
                b = nb;
            else
                A = [A;nA];
                b = [b;nb];
            end
        else
%             sfigure(5);
%             clf;
%             plot(tt,d2th2{i}(ptape),'r',tt,d2y2{i}(ptape),'b',tt,nA*nc,'g');
%             drawnow;
%             pause(0.4);
        end
    end
end
clear ptape;
%%
rel=A(:,[1 4])\A(:,2)
norm(A(:,[1 4])*rel-A(:,2),2)/...
norm(A(:,2),2)
%%
cols = [1 2 5];
cond(A(:,cols))
coefs = A(:,cols)\b;
%%
[norm(A(:,cols)*coefs-b,2)...
    %norm(A(:,cols)*coefs2-b,2)...
    norm(b,2)]
%%
N=numel(t2);
match=zeros(size(t2));
sigma = 0.4;
for i=1:N
    match(i) = sum(exp(-(t2-t2(i)).^2/sigma^2).*dy2);
end
alpha = 0.00425*0.4/sigma;
match = alpha * match;
%%
figure(1)
hold off
plot(t2,dy2,t2,match)
%%
figure(1)
hold on
NUM_TRIALS = 5;
A = cell(NUM_TRIALS,1);
b = cell(NUM_TRIALS,1);

for i=2:N-1
    extreme = 0;
    coef = 0;
    if match(i) > 0.013 && match(i) > match(i-1) && match(i) > match(i+1)
        extreme = 1;
        coef = 1;
    elseif match(i) < -0.013 && match(i) < match(i-1) && match(i) < match(i+1)
        extreme = 1;
        coef = -1;
    end
    
    if extreme
        plot(t2(i),match(i),'ro');
        nA = coef*[th2(i-60:i+60) dth2(i-60:i+60) y2(i-60:i+60) dy2(i-60:i+60) d2y2(i-60:i+60)];
        nb = coef*[d2th2(i-60:i+60)];
        for j=1:NUM_TRIALS
            if rand < 0.7
                if numel(A{j}) == 0
                    A{j} = nA;
                    b{j} = nb;
                else
                    A{j} = A{j} + nA;
                    b{j} = b{j} + nb;
                end
            end
        end
    end    
end
A2 = A;
b2 = b;
A = A{1};
b = b{1};
for i=2:NUM_TRIALS
    A = [A;A2{i}];
    b = [b;b2{i}];
end
%%
trial = 5;
name = {'thetaddot'};
ptape=121*(trial-1)+1:121*trial;
sfigure(5);
clf;
hold on;
if contains(name,'theta')
    plot(A(ptape,1));
end
if contains(name,'thetadot')
    plot(A(ptape,2));
end
if contains(name,'y')
    plot(A(ptape,3));
end
if contains(name,'ydot')
    plot(A(ptape,4));
end
if contains(name,'yddot')
    plot(A(ptape,5));
end
if contains(name,'thetaddot')
    plot([b(ptape) A(ptape,cols)*coef]);
end

%%
cond(A(:,[1 2 4 5]))
%%
cols = [1 2 4 5];
coef = A(:,cols)\b;
%coef(2) = -3;
coef
sfigure(3);
clf;
plot([A(:,cols)*coef b])
%%
mfreq = 1000;
dth3 = cell(K,1);
for i=1:K
    foo=dth2{i};
    foohat=fft(foo);
    foohat(mfreq+2:end-mfreq) = 0;
    foo2=ifft(foohat);
    dth3{i}=real(foo2);
    size(dth2{i})
    size(dth3{i})
end
%%
sfigure(2);
plot([foo real(foo2)]); axis([5500 7500 -0.45 0.6])