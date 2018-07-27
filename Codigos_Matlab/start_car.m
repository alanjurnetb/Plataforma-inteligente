function start_car(s)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    fprintf(s,'G1');  
    out=cellstr('1');
    while (~strcmp(out{1},'Ready'))
        out = cellstr(fscanf(s))
    end

end

