coefs = A(:,[1 2 4 5])\b
%%
%for trial=1:50

dt = 0.003;
K = 4; A = rand(K,K)-K*eye(K); B = rand(K,1);
A =[[-0.25 -0.2 0.4 0.2];[0.1 -0.53 -0.3 0];[0.01 0 -0.2 0.3];[0.2 0.38 -0.24 -0.42]];%[[-0.4 0.2 0.8];[0.1 -0.3 0.2];[0.1 0.2 0.1]];
B = 5*[0.1;-0.2;0.3;0.1];%[0.2;1.1;-0.2];
A2=A; B2=B;
sys = c2d(ss(A,B,eye(size(A)),[]),dt);
%sys
ttape = (0:dt:10)';
N = numel(ttape);
utape = zeros(size(ttape,1),1);
ranges = 0.5*[[0 1];[2 4.5];[8.3 11];[14 17.5]];
coefs = [1;-2;5;-3];
K = size(ranges,1);
for i=1:K
    s = round(ranges(i,1)/dt)+1;
    e = round(ranges(i,2)/dt);
    utape(s:e) = utape(s:e)+coefs(i);
end
utape = chirp(ttape,0.1,30,1,'linear');
%utape(3*end/4:end)=0;
sfigure(2);
plot(ttape,utape)
%for i=2:N
%    utape(i,:) = utape(i-1,:)+(2*rand(1,2)-1)*diag([1.0 0.03]);
%end
xtape=lsim(sys,utape,ttape);
sfigure(1);
lsim(sys,utape,ttape)

P=canon_proj_discrete_stable(xtape,dt);
A=sys.a;
B=sys.b;
syscan = c2d(canonical(sys),dt);
%ns(trial) = norm(P*A*inv(P)-syscan.a,2);
%nsys{trial} = sys;
%end
P*A*inv(P)
P*B
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