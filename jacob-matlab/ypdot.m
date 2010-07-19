function ypdot=ypdot(t,yp,theta)
    global ctltape dt;
    ypdot = ctltape(1+round(t/dt));
return