function v = ls2(A,B)
    % two-sided least squares
    % minimizes ||Ax-By|| subject to ||y|| = 1
    X = [A -B]'*[A -B];
    Y = diag([zeros(1,size(A,2)) ones(1,size(B,2))]);
    save X.dat X
    [v,d] = eig(X,Y,'qz');
    [lambda,ind] = min(abs(diag(d)));
    lambda = lambda(1); ind = ind(1);
    fprintf(1,'Squared error: %e\n',lambda);
    v=v(:,ind);
end