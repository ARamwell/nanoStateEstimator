function [elapTime] = refTimeToElapsedTimeDouble(inTime, refTime)
%REFTIMETOELAPSEDTIMEDOUBLE Summary of this function goes here
%   Detailed explanation goes here
    elapTime = createArray(1,1);
    for t = 1:size(inTime, 2)
        %newTime(t) = datetime(refTime+seconds(simTime(t)), 'Format', format);
        elapTime(1,t) = seconds(inTime(1,t) - refTime);
    end

end

