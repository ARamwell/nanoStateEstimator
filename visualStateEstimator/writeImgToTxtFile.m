function writeImgToTxtFile(img, file)

    %fileID = fopen("/home/ros2_ws/outputs/testOutput.txt", 'w');
    fileID = fopen(file, 'w');

    %fprintf(fileID, '%f,', 9);
    imageSize = (size(img));
    
    fprintf(fileID, '%d,', int32(imageSize(1)));
    fprintf(fileID, '%d', int32(imageSize(2)));
    fprintf(fileID, '\n');

    if size(imageSize) == 3
        for d=1:imageSize(3)
            for r=1:size(img, 1, d) %rows
                for c=1:size(img, 2) %columns
                    fprintf(fileID, '%d,', int32(img(r,c,d)));
                end
                fprintf(fileID, '\n');
            end
        end
          
    else
        for r=1:size(img, 1) %rows
            for c=1:size(img, 2) %columns
                fprintf(fileID, '%d,', int32(img(r,c)));
            end
            fprintf(fileID, '\n');
        end
    end
    
    fclose(fileID);
    
end