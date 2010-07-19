global ctltape dt;
dt = 0.0005;
N = 10;
T = 5.0;
xtape = zeros(N,round(T/dt));
vtape = zeros(N,round(T/dt));
ctape = zeros(N,round(T/dt));
ttape = zeros(N,round(T/dt));
for i=1:N
    [xtape0,vtape0,ctape0,ttape0] = idtape(T,6);
    xtape(i,:) = xtape0;
    vtape(i,:) = vtape0;
    ctape(i,:) = ctape0;
    ttape(i,:) = ttape0;
    ctltape = vtape0;
    PDEFlap(pi-1+2*rand, rand, T);
    cmd1 = sprintf('mkdir -p run%d/',i);
    cmd2 = sprintf('mv *.txt run%d/',i);
    system(cmd1);
    system(cmd2);
end

%% get y, theta

thetatape = zeros(N,round(T/dt));
ytape = zeros(N,round(T/dt));
for i=1:N
    fname = sprintf('run%d/thetaout.txt',i);
    fid=fopen(fname,'r');
    thetatape(i,:)=fscanf(fid,'%22f'); thetatape(i,:) = mod(thetatape(i,:),2*pi) - pi; % recenter about 0, since the downright is normally about pi
    fclose(fid);

    fname = sprintf('run%d/ypout.txt',i);
    fid=fopen(fname,'r');
    ytape(i,:)=fscanf(fid,'%22f');
    fclose(fid);

    thetatape(i,:) = unmod(thetatape(i,:),2*pi); % get rid of discontinuities that come from modding out by 2pi; this could conceivably make the data no longer centered about the desired point
    ytape(i,:) = unmod(ytape(i,:),2*pi);
end

%% use acausal derivative filter to get ydot, thetadot
ydottape = zeros(N,round(T/dt));
thetadottape = zeros(N,round(T/dt));
for i=1:N
    ydottape(i,:) = df(ytape(i,:))/dt;
    thetadottape(i,:) = df(thetatape(i,:))/dt;
end

%% resize to make up for derivative filter cutoff
ttape = ttape(:,5:end-4);
ytape = ytape(:,5:end-4);
thetatape = thetatape(:,5:end-4);
thetadottape = thetadottape(:,5:end-4);
ydottape = ydottape(:,5:end-4);
utape = ctape(:,5:end-4); % don't overwrite ctape since it is expensive to generate

thetatape = mod(thetatape+pi,2*pi)-pi; % make theta small

%% create linear model
% thetadot = x1*theta + x2*thetadot + x3*u is the only term that we need to
% fit
A = [tocol(thetatape(:,1:end-1)) tocol(thetadottape(:,1:end-1)) tocol(ydottape(:,1:end-1)) tocol(utape(:,1:end-1))];
b = tocol(thetadottape(:,2:end));
x=A\b;
x
xcoef = [[1 dt 0 0];[x(1) x(2) 0 x(3)];[0 0 1 dt];[0 0 0 1]]
ucoef = [0;x(4);0;dt]

%% make it into a CT model
sys = ss(xcoef,ucoef,eye(4),[],dt);
sysc = d2c(sys);

%% invert, get controller about upright
[a1,b1,c1,d1]=ssdata(sysc);
m=max(real(eigs(a1)));
if(m > 0)
    display('Warning: open-loop system has an eigenvalue with positive real part.');
    display('Eigenvalues:');
    disp(eigs(a1));
end
a2 = [[0 1 0 0];[-a1(2,1) a1(2,2) 0 -a1(2,4)];[0 0 0 1];[0 0 0 0]];
b2 = [0;-b1(2);0;1];
sys2 = ss(a2,b2,eye(4),[]);

k=lqr(sys2,[[10 0 0 0];[0 1 0 0];[0 0 40 0];[0 0 0 4]],0.1)
