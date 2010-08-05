% smooth
%
% [f,d] = smooth(y,k,alpha,w)
%
%  Computes:  min_f  ||y-f||_2^2 + alpha^2||WDf||_2^2
%       W = diag(w)
%       D = k-th difference operator
%
%  y     -- n-by-1 signal to smooth
%  k     -- order of derivative to regularize
%  alpha -- (n-k)-by-1 weights on derivative regularization.
%  w     -- element-wise weights on regularization (default ones)
%
% Returns:
%  f     -- the fit.
%  d     -- Df (k-th difference of f).
%
function [f,d] = smooth(y,k,alpha,w)
    if nargin < 4
        w = [];
    end

    N = length(y);

    
     P = pascal(k+1,2);
     E = ones(N,1)*P(:,1)';
     D = spdiags(E,0:k,N-k,N);
    
     if isempty(w)
         w = ones(N-k,1);
     end
     
     Gamma = alpha^2*D'*spdiags(w.^2,0,N-k,N-k)*D;
     f = (speye(N)+Gamma)\y;
     d = D*f;
end