function b=equals(str1,str2)
    b = numel(str1) == numel(str2) && all(str1 == str2);
end