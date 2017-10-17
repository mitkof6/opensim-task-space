function [ index ] = findStr(data, pat)
%FINDSTR Finds the index of a pattern from a string cell vector

index = find(not(cellfun('isempty', strfind(data, pat))));

end

