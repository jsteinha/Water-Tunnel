function dy = diffs(y,p,dt)
    N = numel(y);
    dy = zeros(size(y));
    for k=1:min([N-p p-1 400])
        dy(k) = (y(p+k)-y(p-k))/(2*k*dt);
    end
end