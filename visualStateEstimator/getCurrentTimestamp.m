function [sec, nsec] = getCurrentTimestamp()
%DATETIMETOROS2TIME Summary of this function goes here
%   Detailed explanation goes here


currentTime = datetime('now');
totalSecondsSinceEpoch = posixtime(currentTime);


sec = int32(floor(totalSecondsSinceEpoch));
totalNanoSecondsSinceEpoch = (totalSecondsSinceEpoch - sec)*1e9;
nsec = uint32(floor(totalNanoSecondsSinceEpoch));


end

