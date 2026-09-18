function [selectedIndexArray] = selectClosestTimeIndices(arrayToMatch,arrayToSelectFrom)
%SELECTCLOSESTTIMES Summary of this function goes here
%   Arrays must have time in first row, or be time row vectors. (i.e., time
%   increments horizontally)
%   Detailed explanation goes here

selectedIndexArray = createArray(size(arrayToSelectFrom, 1), size(arrayToMatch, 2));


for t = 1:size(arrayToMatch, 2)

    t_target = arrayToMatch(1, t); %could be a datetime, could be a float

    if contains(class(t_target), 'datetime')
        [closestDiff, closestIndex] = min(arrayToSelectFrom(1,:)-t_target);
    else
        [closestDiff, closestIndex] = min(abs(arrayToSelectFrom(1,:)-t_target));
    end

    selectedIndexArray(:, t) = closestIndex;

end

