function [dec, sec, nsec] = getCurrentTimestamp()
%DATETIMETOROS2TIME Summary of this function goes here
%   Detailed explanation goes here


currentTime = datetime('now');
%currentTime.Format = 'dd-MMM-yyyy HH:mm:ss.SSSSSSSSS';


totalSecondsSinceEpoch = posixtime(currentTime);

secondsSinceEpoch = (floor(totalSecondsSinceEpoch));
totalNanoSecondsSinceEpoch = (totalSecondsSinceEpoch - secondsSinceEpoch)*1e9;

sec = uint32(secondsSinceEpoch);
nsec = uint32(floor(totalNanoSecondsSinceEpoch));
dec = totalSecondsSinceEpoch;


end

