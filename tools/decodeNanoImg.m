fileList = selector_multiFile();

%[fileName, fileDir] = uigetfile();
%fullDir = strcat(fileDir, fileName);

for i = 1:length(fileList)
    fullDir = fileList{i};

    %read file
    txtTable = readtable(fullDir, "Delimiter", ',', 'NumHeaderLines', 0);
    
    imgSize = txtTable{1, 1:2};
    
    img = uint8(txtTable{2:end, :});
    
    %imshow(img);
    img_rgb = cat(3, img, img, img);
    
    imwrite(img_rgb, strcat(fullDir, '.png'));
end


%imag = imread('C:\Users\Alyssa\Documents\QuadStateEstimator\QuadSimEnv\Results\Traj-0026\000500.jpg');