function ds = df(s)
    taps = -[0.003059 0.035187 0.118739 0.143928 0.000000 -0.143928 -0.118739 -0.035187 -0.003059];
    ds = zeros(size(s));
    for i=5:max(size(s))-4
       ds(i) = sum(s(i-4:i+4).*taps);
    end
end