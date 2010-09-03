function P=canon_proj_discrete(x)
    % xtape should be an NxD vector, N = num_samples, D=dimension
    N = size(x,1);
    D = size(x,2);
    y = zeros(D-1,N-D+1,D);
    for i=1:D-1
        y(i,:,:) = x(D-i:N-i,:);
    end
    x = x(D:N,:);
    
   
    A = kron(eye(D-1),x);
    B = reshape(permute(y,[2 1 3]),[(D-1)*(N-D+1) D]); % [y_1;y_2;y_3;...;y_{d-1})]
    v = ls2(A,B);
    %v = v(1:end); % actual transition matrix
    P = reshape(v,[D D])';
    P = P([D 1:D-1],:); % since we solved for projections onto 
                        %y1,y2,...,y_{d-1},x instead of x,y1,y2,...,y_{d-1}
%    x0=c;
%    x0(1) = x0(1)+P(1,:)*x(1,:)';
end