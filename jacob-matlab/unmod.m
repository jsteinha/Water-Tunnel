function ns = unmod(s,m)
    tol = 2e-2;
    for i=2:max(size(s))
       while s(i)-s(i-1) > tol
           s(i) = s(i) - m;
       end
       while s(i)-s(i-1) < -tol
           s(i) = s(i) + m;
       end
       if s(i)-s(i-1) > m/2
           s(i)-s(i-1)
           s(i) = s(i) - m;
       end
       if abs(s(i)-s(i-1)) > tol
           display('WARNING: Could not satisfy tolerances.');
           i
       end
    end
    ns = s;
end