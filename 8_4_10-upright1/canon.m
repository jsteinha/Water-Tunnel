coefs = A(:,[1 2 4 5])\b
%%
%for trial=1:50

dt = 0.003;
A = rand(5,5)-3*eye(5);%[[-0.4 0.2 0.8];[0.1 -0.3 0.2];[0.1 0.2 0.1]];
%while max(abs(eig(A))) > 0.99
%    A = rand(5,5)/3;
%end
B = rand(5,1);%[0.2;1.1;-0.2];
sys = c2d(ss(A,B,eye(size(A)),[]),dt);
%sys
ttape = (0:dt:10)';
N = numel(ttape);
utape = zeros(size(ttape,1),1);
ranges = 0.5*[[0 1];[2 4.5];[8.3 9];[16 16.5]];
coefs = [1;-1;1;-1];
K = size(ranges,1);
for i=1:K
    s = round(ranges(i,1)/dt)+1;
    e = round(ranges(i,2)/dt);
    utape(s:e) = utape(s:e)+coefs(i);
end
sfigure(2);
plot(ttape,utape)
%for i=2:N
%    utape(i,:) = utape(i-1,:)+(2*rand(1,2)-1)*diag([1.0 0.03]);
%end
xtape=lsim(sys,utape,ttape);
sfigure(1);
lsim(sys,utape,ttape)

P=canon_proj_discrete(xtape);
A=sys.a;
B=sys.b;
syscan = c2d(canonical(sys),dt);
%ns(trial) = norm(P*A*inv(P)-syscan.a,2);
%nsys{trial} = sys;
%end
%%
ytape = cumtrapz(ttape,xtape);
sfigure(1);
plot(ttape,xtape);
sfigure(2);
plot(ttape,ytape);
%%
M = [[xtape'*xtape xtape'*ytape];[ytape'*xtape ytape'*ytape]];
N = diag([0 0 -1 -1]);
[V,D]=eig(M,N)
w = V(1:2,3);
v = -V(3:4,3);
%%
P = [v';w'];
A2=P*A/P;
B2=P*B;
A2
B2