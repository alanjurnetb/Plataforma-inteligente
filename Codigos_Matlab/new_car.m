function [ s ] = new_car( port )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    s = serial(port)
    set(s,'BaudRate',250000)
    fopen(s)
    
    out=cellstr('1');
    while (~strcmp(out{1},'Ready'))
        out = cellstr(fscanf(s))
    end

end

