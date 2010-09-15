function P=canon_proj_discrete_stable(x,dt)
    % xtape should be an NxD vector, N = num_samples, D=dimension
    N = size(x,1);
    D = size(x,2);
    
    A = kron(eye(D-1),x(2:N,:)-x(1:N-1,:)) + ...
        kron([zeros(1,D-1);eye(D-2,D-1)],-dt * x(1:N-1,:));
    
    B = zeros((N-1)*(D-1),D);
    B(1:N-1,:) = dt * x(1:N-1,:);

    size(A)
    size(B)
    
    v = ls2(A,B);
    P = reshape(v,[D D])';
    P = P([D 1:D-1],:); % since we solved for projections onto 
                        %y1,y2,...,y_{d-1},x instead of x,y1,y2,...,y_{d-1}
    
end