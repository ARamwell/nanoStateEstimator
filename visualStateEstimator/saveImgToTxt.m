function saveImgToTxt(img, name_postscript)
%NANOIMGTOTXT Summary of this function goes here
%   Detailed explanation goes here
%#codegen

    hwobj = jetson;
    %camlist = getCameraList(hwobj);
    camName = 'vi-output, imx219 6-0010';
    %camRes = [1280 720];
    camRes = [640 360];
    %K = [384.5970 0 159.4752; 0 385.8926 338.3277; 0 0 1];
    
    
    camObj = camera(hwobj,camName,camRes);
    dispObj = imageDisplay(hwobj); %for troubleshooting

    for i=1:35
        img = ((snapshot(camObj)));
        
        img_g = rgb2gray(img);
      
        image(dispObj, img_g);
    
        file = strcat('testimg_', sprintf( '%02d', i ), '.txt');

        writeImgToFile(img_g, file);
        
        pause(5);
    end
  


end

function writeImgToFile(img, file)

    %fileID = fopen("/home/ros2_ws/outputs/testOutput.txt", 'w');
    fileID = fopen(file, 'w');

    %fprintf(fileID, '%f,', 9);
    imageSize = (size(img));
    
    fprintf(fileID, '%d,', int32(imageSize(1)));
    fprintf(fileID, '%d', int32(imageSize(2)));
    fprintf(fileID, '\n');


    for r=1:size(img, 1) %rows
        for c=1:size(img, 2) %columns
            fprintf(fileID, '%d,', int32(img(r,c)));
        end
        fprintf(fileID, '\n');
    end
    
    fclose(fileID);
    
end