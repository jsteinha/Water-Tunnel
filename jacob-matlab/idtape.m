function [xtape,vtape,utape,ttape] = idtape(T,K)
%
dt = 0.0005;
Amin = -0.6;
Amax = 0.6;
times = [0 sort(T*rand(1,K-1)) T];
acc = Amin+(Amax-Amin)*rand(1,K);
len = times(2:K+1)-times(1:K);
ttape = 0:dt:T-1e-10;
ptape = zeros(size(ttape));
qtape = zeros(size(ttape));
atape = zeros(size(ttape));
%
for i=1:K
    kmin = 1;
    kmax = size(ttape,2);
    while ttape(kmin) < times(i)
        kmin = kmin + 1;
    end
    while ttape(kmax) > times(i+1)
        kmax = kmax-1;
    end
    x = 0.5*sum(acc(1:i-1).*len(1:i-1).^2)+sum(acc(1:i-1).*len(1:i-1).*(times(i)-times(2:i)));
    y = sum(acc(1:i-1).*len(1:i-1));
    z = 0.5*acc(i);
    f=@(t)x+y*(t-times(i))+z*(t-times(i))^2;
    g=@(t)sum(acc(1:i-1).*len(1:i-1))+acc(i)*(t-times(i));
    for k=kmin:kmax
        ptape(k) = f(ttape(k));
        qtape(k) = g(ttape(k));
        atape(k) = acc(i);
    end
end
%
xtape = cos(ptape);
vtape = -qtape.*sin(ptape);
utape = -atape.*sin(ptape)-qtape.^2.*cos(ptape);
%plot(ttape,xtape,ttape,vtape,ttape,utape);
end