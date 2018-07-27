function wait_connection(s)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    out=cellstr('1');
    while (~strcmp(out{1},'OK'))
        out = cellstr(fscanf(s))
    end
end