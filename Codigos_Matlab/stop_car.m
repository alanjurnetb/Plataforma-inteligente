function stop_car(s)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    fprintf(s,'K1');  
    out=cellstr('1');
    while (~strcmp(out{1},'Ready'))
        out = cellstr(fscanf(s))
    end

    fprintf(s,'R1'); 
    out=cellstr('1');
    while (~strcmp(out{1},'Ready'))
        out = cellstr(fscanf(s))
    end

    fprintf(s,'C1'); 
    out=cellstr('1');
    while (~strcmp(out{1},'Ready'))
        out = cellstr(fscanf(s))
    end
    
end

