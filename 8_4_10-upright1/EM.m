%% EM algorithm
function EM(data,numPeaks)
    % data should be a row vector
    assert(numel(size(data))==2 && size(data,2)==1,'data must be a row vector');
    N = size(data,1);
    mu = (N+1)*rand(1,numPeaks);
    sigma = abs(normrnd(0,100,[1 numPeaks]));
    alpha = 0.02*ones(1,numPeaks);
    for iter = 1:100
        iter
        mu
        sigma
        alpha
        figure(1);
        hold off;
        plot(data,'b');
        hold on;
        approx = zeros(N,1);
        for i=1:numPeaks
            approx = approx + alpha(i) * exp(-(mu(i)-(1:N)').^2/sigma(i)^2);
        end
        plot(approx,'r');
        drawnow;
        
        weights = findWeights(mu,sigma,alpha,data,N,numPeaks);
        figure(2);
        plot(weights);
        [mu,sigma,alpha] = findPeaks(data,weights,N,numPeaks);
        pause(1.0);
    end
end

function weights = findWeights(mu,sigma,alpha,data,N,P)
    weights = zeros(N,P);
    for i=1:N
        line = (data(i)-alpha.*exp(-(mu-i).^2./sigma.^2)).^2;
%         if i < 10
%             line
%         end
        base = min(line);
        weights(i,:) = exp(200*(base-line));
        weights(i,:) = weights(i,:) / sum(weights(i,:));
    end
end

function [mu,sigma,alpha] = findPeaks(data,weights,N,P)
    mu = zeros(1,P);
    sigma = zeros(1,P);
    alpha = zeros(1,P);
    for i=1:P
        wt2 = weights(:,i); % .* abs(data);
        sw2 = sum(wt2);
        mu(i) = sum(wt2 .* (1:N)')/sw2;
        sigma(i) = sum(wt2 .* ((1:N)'-mu(i)).^2)/sw2;
        alpha(i) = 0.02; %sum(wt2 .* data) / (sw2*sqrt(2*pi*sigma(i)^2));
    end
end