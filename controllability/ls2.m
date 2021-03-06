function v = ls2(A,B)
    % two-sided least squares
    % minimizes ||Ax-By|| subject to ||y|| = 1
    GLS = 0;
    if GLS
        X = [A -B]'*[A -B];
        Y = diag([zeros(1,size(A,2)) ones(1,size(B,2))]);
        save X.dat X
        X
        Y
        [v,d]=eig(X,Y,'qz');
    else
        %P = A'*A; Q = -A'*B; R = -B'*A; S = B'*B;
        M = B'*(B-A*(A\B)); % B'B-B'A(A'A)^(-1)A'B
%        M
        [v,d]=eig(M);
    end
%    v
    format long e
    diag(d)
    format short
    [lambda,ind] = min(abs(diag(d)));
    lambda = lambda(1); ind = ind(1);
    fprintf(1,'Squared error: %e\n',lambda);
    for i=1:size(v,1)
        x=A\(B*v(:,i)); y=v(:,i);
%        norm(A*x-B*y,2)
    end
    v=v(:,ind);
    v=[A\(B*v);v];
end