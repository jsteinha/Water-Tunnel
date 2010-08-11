%
%  f = smoothing(y,k,m,alpha,l)
%
%  Iteratively smoothes.
%
%  The i-th iteration computes:
%
%      min_(f_i)  ||y-f_i||_2^2 + alpha^2 ||W_i Df_i||_2^2
%
%  Where D is the k-th difference matrix, and W_i is a square, diagonal weight matrix.
%
%  W_0     = eye(n-k).
%  beta_0  = 1
%
%  d_i     = (Df_i);
%  beta_i  = l*beta;
%  w_{i+1}(j)  = 1/(1 + |d_i(j)|)^(beta_i)
%  W_{i+1} = diag(w_{i+1})
%
%
%  y     -- n-by-1 signal to be smoothed
%  k     -- derivative to regularize. (default 4)
%  m     -- number of iterations (default 4)
%  alpha -- weight for regularization (default 1e2)
%  l     -- controls adjustment of weights (default 4)
%
function f = smoothing(y,k,m,alpha,l)
    w = [];
    beta = 1;
    if nargin < 2
        k = 4;
    end
    if nargin < 3
        m = 4;
    end
    if nargin < 4
        alpha = 1e2;
    end
    if nargin < 5
        l = 4;
    end
    
    for i = 1:m
        [f,d] = smooth(y,k,alpha,w); 
        beta = l*beta; dnorm = d;%/max(abs(d));
        w = 1./(abs(dnorm)+1).^beta;
    end
end