function set_goal(s,point)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    fprintf(s,'X%.3f',point(1));  
    out=cellstr('1');
    while (~strcmp(out{1},'Ready'))
        out = cellstr(fscanf(s))
    end
    fprintf(s,'Y%.3f',point(2));  
    out=cellstr('1');
    while (~strcmp(out{1},'Ready'))
        out = cellstr(fscanf(s))
    end
    fprintf(s,'S1');  
    out=cellstr('1');
    while (~strcmp(out{1},'Ready'))
        out = cellstr(fscanf(s))
    end
end

