function b=contains(arr,str)
    b = 0;
    for i=1:numel(arr)
        b = b || equals(arr{i},str);
    end
end