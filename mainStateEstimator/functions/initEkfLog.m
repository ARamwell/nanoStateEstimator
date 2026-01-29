function fileID = initEkfLog(fileName)
%INITEKFLOG Summary of this function goes here
%   Detailed explanation goes here
    %open file
    fileID = fopen(sprintf('test_name%s.txt', fileName) ,'w');

    %print headers
    %headers = {"timestamp", "timeSinceLastCorrection", repmat("xPost", 1,16), repmat("u",1,6), repmat("z",1,7),  repmat("xHat",1,16), repmt("zHat",1,7), repmat("y",1,7), repmat("PHat",1,136), repmat("PPost",1,136), repmat("S",1,28), repmat("K",1,112), repmat("W",1,49), repmat("Q",1,16)};
    headers = ["timestamp", "timeSinceLastCorrection", repmat("xPost", 1,16), repmat("u",1,6), repmat("z",1,7), repmat("y",1,7), repmat("PHat",1,136), repmat("S",1,28), repmat("W",1,49)];
    for r=1:size(headers, 1)
        for c=1:size(headers, 2)
            fprintf(fileID, '%s,', headers{r,c});
        end
        fprintf(fileID, '\n');
    end
end
