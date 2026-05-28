
folders = selector_multiFolder;
convGTFrame =false;
doSave =false;

%%
for f=1:length(folders)
    folder_f = folders{f};

    fileName = strcat(folder_f, "\ekfLog.txt");

    %fileName ="C:\Users\Alyssa\Documents\ekfLog.txt";
    
    ekfResult = replicateSimLogs(fileName, convGTFrame);

    if doSave
        saveName = strcat(folder_f, "\ekfResult_16el_rect_a0.mat");
        save(saveName, '-struct', 'ekfResult');
    end
end

