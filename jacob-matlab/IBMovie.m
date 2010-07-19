quiversurf = 2; % 1 if quiver plot movie is desired. 2 if surface movie is desired.


fid=fopen('misc.txt','r');
Miscin=fscanf(fid,'%22f');
fclose(fid);

N=Miscin(1);
S=Miscin(2);
T=Miscin(3);
dt=Miscin(4);
Poften=Miscin(5);

h=2*pi/N;

tt=linspace(0,T*dt,T+1);
x=0:h:h*(N-1);
y=x;

Nshiftl=[N 1:N 1 2];
Nshiftr=[2:N 1 2];
Sshiftl=[S 1:S-1 1 2];
Sshiftr=[2:S 1 2];

for j=1:N
    for k=1:N
        Dhatx(j,k)=i/h*sin(2*pi*(j-1)/N);
        Dhaty(j,k)=i/h*sin(2*pi*(k-1)/N);
    end
end


fid=fopen('boundaryX.txt','r');
Xin=fscanf(fid,'%22f');
fclose(fid);

fid=fopen('boundaryY.txt','r');
Yin=fscanf(fid,'%22f');
fclose(fid);

fid=fopen('thetaout.txt','r');
theta=fscanf(fid,'%22f');
fclose(fid);

fid=fopen('ypout.txt','r');
yp0=fscanf(fid,'%22f');
fclose(fid);

fid=fopen('vorticity.txt','r');
uvwin=fscanf(fid,'%22f');
fclose(fid);
maxw=max(max(abs(uvwin)));

Nparts=80;
XParticle(1:80)=pi-.15;
YParticle(1:80)=linspace(.5,2*pi-.5,80);
Particles=0; 

if (Poften>0)
pp=1;
for p=0:pp:T/Poften-1
t=p*dt*Poften;

    X=Xin(p*(2*S)+1:p*(2*S)+(2*S));
    Y=Yin(p*(2*S)+1:p*(2*S)+(2*S));
    uN=uvwin(p*(2*(N/2)*N)+1:p*(2*(N/2)*N)+(N/2)*N);
    vN=uvwin(p*(2*(N/2)*N)+(N/2)*N+1:p*(2*(N/2)*N)+2*(N/2)*N);

    u(1:N,1:N)=0;
    v(1:N,1:N)=0;
    u(1:N,N/4+1:3*N/4)=reshape(uN,N,N/2);
    v(1:N,N/4+1:3*N/4)=reshape(vN,N,N/2);
    
    uht=fft2(u);
    vht=fft2(v);
    wht=(Dhatx.*vht-Dhaty.*uht);
    wht(1,1)=0;
    wht(N/2+1,1)=0;
    wht(N/2+1,N/2+1)=0;
    wht(1,N/2+1)=0;
    w=real(ifft2(wht));
    
    %FXbody1=ds*sum(fX(1:S));

if Particles==1
if (p==0)
    DT=dt;
else
    DT=dt*pp*Poften;
end

    
    for l=1:Nparts
        XPnew(l)=XParticle(l);
        YPnew(l)=YParticle(l);
        
        locP=[floor(XParticle(l)/h)+1 floor(YParticle(l)/h)+1];
        j0=locP(1)-1;
        for j=Nshiftl(locP(1)+(0:3))
            k0=locP(2)-1;
            for k=Nshiftl(locP(2)+(0:3))
                XPnew(l)=mod(XPnew(l)+(DT)*(u(j,k)*delta((j0-1)*h-XParticle(l),(k0-1)*h-YParticle(l),h))*h^2,2*pi); 
                YPnew(l)=mod(YPnew(l)+(DT)*(v(j,k)*delta((j0-1)*h-XParticle(l),(k0-1)*h-YParticle(l),h))*h^2,2*pi); 
                k0=k0+1;    
            end
        j0=j0+1;    
        end         
        XParticle(l)=XPnew(l);
        YParticle(l)=YPnew(l);
    end
end

if (quiversurf==1) 
figure(1)
      quiver(x,y,u',v'), shading interp
      hold on
      plot(X(1:S),Y(1:S),'k.')
      plot(X(S+1:2*S),Y(S+1:2*S),'k.')
      plot(X(1),Y(1),'bo')
      if Particles==1
         plot(XParticle(1:80),YParticle(1:80),'k.')
        plot(XParticle(21:40),YParticle(21:40),'k+')
      end
      hold off, axis equal, axis(pi*[1 1 1 1]+pi*[-0.4 0.4 -0.5 0.5])
      title(['t = ' num2str(t)])
      drawnow
      pause(0.2)

elseif (quiversurf==2) 
figure(1)
      surf(x,y,w'), shading interp
      hold on
      plot(X(1:S),Y(1:S),'k.')
      plot(X(S+1:2*S),Y(S+1:2*S),'k.')
      plot(X(1),Y(1),'bo')
      if Particles==1
         plot(XParticle(1:80),YParticle(1:80),'k.')
        plot(XParticle(21:40),YParticle(21:40),'k+')
      end
      hold off, axis equal, axis(pi*[1 1 1 1]+pi*[-0.4 0.4 -0.5 0.5])
      caxis([-maxw maxw+.0001]*4)
      title(['t = ' num2str(t)])
      drawnow
      pause(0.2)
end %quiversurf

end %time

end %Poften>0

figure(3)
clf(3)
plot(tt(2:end),theta),xlabel('t [s]'),ylabel('\theta')

figure(4)
clf(4)
plot(tt(2:end),yp0),xlabel('t [s]'),ylabel('y_p(t)')
