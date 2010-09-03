function [P,x0]=canon_proj(t,x)
    % xtape should be an NxD vector, N = num_samples, D=dimension
    N = size(x,1);
    D = size(x,2);
    y = zeros(D-1,N,D);
    y(1,:,:) = cumtrapz(t,x,1);
    for i=2:D-1
        y(i,:,:) = cumtrapz(t,y(i-1,:,:),2);
    end
    
    f=@(tt,j) tt.^j/factorial(j);
    H = zeros((D-1)*N,D);
    for i=1:D-1
        for j=0:i
            range = (i-1)*N+1:i*N;
            H(range,j+1) = -f(t,i-j);
        end
    end
    A = [H kron(eye(D-1),x)];
    cond(A)
    B = reshape(permute(y,[2 1 3]),[(D-1)*N D]); % [y_1;y_2;y_3;...;y_{d-1})]
    v = ls2(A,B);
    c = v(1:D); % initial conditions
    v = v(D+1:end); % actual transition matrix
    P = reshape(v,[D D])';
    P = P([D 1:D-1],:); % since we solved for projections onto 
                        %y1,y2,...,y_{d-1},x instead of x,y1,y2,...,y_{d-1}
    x0=c;
    x0(1) = x0(1)+P(1,:)*x(1,:)';
end