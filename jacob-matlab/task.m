function [xtape,vtape,ctape,ttape]=task(run)
    global ctltape dt;
    dt = 0.0005;
    T = 8.0;
%    omega = 0.5+2.5*rand;
    [xtape,vtape,ctape,ttape] = idtape(T,7);
    ctltape = vtape;
    PDEFlap(pi-1+2*rand, rand, T, run);
    
end

