
folders = selector_multiFolder_nano;

%%
for f=1:length(folders)

    folder_f = folders{f};

    fileName = strcat(folder_f, "\ekfLog.txt");
    %fileName = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\move11_wToL_extLog\ekfLog.txt";

    [ekfResult_in, groundTruth, trajErr_in] = readEkfLog(fileName, true);

    %%
    %fileName = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\stationary1\data.mat";

    %data = load(fileName, "-mat");

    %%
    u_hist = ekfResult_in.u;
    zArr_hist = ekfResult_in.p3pArr;
    z_hist = ekfResult_in.z;
    t_hist = ekfResult_in.time*(10^-6);
    gt_hist = groundTruth.quad.state;

    %make it impossible to use wrong zArr
    idx_badZ=isnan(z_hist(1,:));
    for t=1:size(zArr_hist,3)
        if idx_badZ(1,t)
            zArr_hist(:,:,t) = nan(7,4);
        end
    end

    mainStateEst_onLogs(u_hist, zArr_hist, t_hist, gt_hist);

    %% Process new log
    ekfLogFile_out = "C:\Users\Alyssa\Documents\nanoStateEstimator\ekfLog.txt";

    ekfResult_out = replicateSimLogs(ekfLogFile_out, false);

    %% Save new result
    saveName = strcat(folder_f, "\ekfResult_16el_rect_a0_reprojOnly.mat");
    save(saveName, '-struct', 'ekfResult_out');

end


