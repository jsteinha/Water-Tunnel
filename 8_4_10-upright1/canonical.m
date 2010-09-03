function sys2 = canonical(sys)
    [a,b,c,d]=ssdata(sys);
    [n,d]=ss2tf(a,b,c,d);
    [a,b,c,d]=tf2ss(n,d);
    sys2=ss(a,b,c,d);
end