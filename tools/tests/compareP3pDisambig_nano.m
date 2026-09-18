
    %featMap = load('C:/Users/Alyssa/Documents/QuadStateEstimator/Resources/featureMap.mat');
    %srcFile = fullfile("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-04-13_10-16-57\sim_spiral");
    % T_rq2rc =  [   0   -1.0000      0    0; ...
    %         1.0000    0         0    0;...
    %         0         0    1.0000    0; ...
    %         0         0         0    1.0000];
   % mocapFile = fullfile("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\physical\testImgsForP3PDebug\mocapHist.mat");
   % p3pSelFile = fullfile("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\physical\testImgsForP3PDebug\p3pSelHist.mat");
   % p3pArrFile = fullfile("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\physical\testImgsForP3PDebug\p3pArrHist.mat");

    %%

    featMap = load("C:\Users\Alyssa\Documents\nanoStateEstimator\visualStateEstimator\featureMap.mat");
    srcFile = fullfile("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\physical\p3pDebug\stationary4_newCamCalib");
    %srcFile = fullfile("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\physical\p3pDebug\moving6_newCamCalib_rebuld_forreport");
    %srcFile =fullfile('C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\physical\p3pDebug\moving4_adjusted_stillOffset');
    %srcFile=("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-04-13_10-16-57_spiral_scaledNoise_px4\sim_spiral");
     % 
     % T_rq2rc =  [   0   -1.0000         0    0.0; ...
     %                1         0         0    0.0;...
     %                0         0    1.0000    0.0; ...
     %                 0         0         0    1.0000]; %sim
    % 
    T_rq2rc = [ 0.0385   -0.9988    0.0299    0.0395; ...
                0.9991    0.0390    0.0135   -0.0112; ...
               -0.0147    0.0294    0.9995    0.1224; ...
                0         0         0         1.0000]; %phys
    % 
    %K = [  576.87         0  338.68;   0  576.86  180.90;   0         0    1.0000]; %640p
    K =  [583.6734         0  309.7243;
         0  582.8750  183.5180;
         0         0    1.0000];


    refTime = datetime(2000, 01, 01); %if importing simulation data

    %% Settings
    inlierThreshold_strict = 20; %20

    %% import images
    [imageStream, imageTime] = imgFuncs.importImageSeq(srcFile, 0, refTime, ".png"); %returns grayscale

    %[imageStream, imageTime] = imgFuncs.importImageSeq(srcFile, 0, refTime, ".jpg"); %returns grayscale
    totalFrames = size(imageTime,2);
    %t_p3p = nan(1, totalFrames);
    %for i = totalFrames
    %    t_p3p=refTimeToElapsedTimeDouble(imageTime, refTime);
    %end

    %% initialise variables
    pq_arr_full = nan(7,4,totalFrames);

    pose_mostInliers = nan(7, totalFrames);
    numIn_mostInliers = nan(1, totalFrames);
    idxSel_mostInliers = nan(1, totalFrames);

    pose_minReproj = nan(7, totalFrames);
    err_minReproj = nan(1, totalFrames);
    idxSel_minReproj = nan(1, totalFrames);

    
    pose_best = nan(7, totalFrames);
    pose_err = nan(3, totalFrames);
    idxSel_best = nan(1, totalFrames);

    pose_statusQuo = nan(7, totalFrames);
    %%err_minReproj = nan(1, totalFrames);
    idxSel_statusQuo = nan(1, totalFrames);

    %% import groundtruth
  %  load(mocapFile);
    pose_true = convertMocap2World(mocap_pq_Hist);
    % fps = 20;
    % simHz =8000;
    % idxImgTime=1:400:(size(simout.simout.quadState.time));
    % pose_true = simout.simout.quadState.signals.values(idxImgTime',1:7)';


    %% Run P3P - nano version
    for i=1:totalFrames
        Rt_arr = nan(3,4,4);
        pq_arr = nan(7,4);
        pq_arr_quad = nan(7,4);
        img = imageStream(:,:,i);
        x_det = nan(1,1);    

        [x_det, X_W_det, id_det] = featureDetectMatch_nano(img, featMap.featureMap); %detect Aruco tags
        x_train = x_det(:,:,1); 
        [x_train, XW_train, id_train, x_test, XW_test, id_test] = featSelect(x_det, X_W_det, id_det', 1); %select features
        
        if ~isnan(x_det(1,1))
            Rt_arr = kneipWrapper(x_train, XW_train, K); %run pose estimator - gives up to four solutions, returns NaN if it can't see any features
            for n=1:size(Rt_arr, 3)
                pq_arr(:,n) = rtToPose(Rt_arr(:,:,n));
                pq_arr_quad(:,n)=tFormPQRight(pq_arr(:,n), invertT(T_rq2rc));
            end 
            pq_arr_full(:,:,i) = pq_arr_quad;

                       
            if ~isnan(Rt_arr(1,1,1))   
                %***** STATUS QUO ********
                [Rt_statusQuo, idxSel_statusQuo(:,i)] = poseDisambig(K, Rt_arr, x_train, XW_train, x_test, XW_test);
                pose_statusQuo(:,i) =pq_arr_quad(:,idxSel_statusQuo(:,i));


                %***** MOST INLIERS + LEAST REPROJ WHEN NEEDED
                if ~isnan(x_test(1,1))
                    [Rt_mostInliers, numIn_mostInliers(:,i), idxSel_mostInliers(:,i)] = chooseRtWithMostInliers(K, Rt_arr, x_test, XW_test, inlierThreshold_strict);  
                    % if numIn_mostInliers(:,i)<1 % %relax inlier threshold if it fails
                    %      [Rt_mostInliers, numIn_mostInliers(:,i), idxSel_mostInliers(:,i)] = chooseRtWithMostInliers(K, Rt_arr, x_test, XW_test, inlierThreshold_relaxed);  
                    % end
                else
                    [Rt_mostInliers, numIn_mostInliers(:,i), idxSel_mostInliers(:,i)]=chooseMinReprojErrW2C(K, Rt_arr, x_train(:,4), XW_train(:,4));
                end
                pose_mostInliers(:,i) =pq_arr_quad(:,idxSel_mostInliers(:,i));

                %***** LEAST REPROJ
                [Rt_minReproj, err_minReproj(:,i), idxSel_minReproj(:,i)]=chooseMinReprojErrW2C(K, Rt_arr, x_train(:,4), XW_train(:,4));
                pose_minReproj(:,i) =pq_arr_quad(:,idxSel_minReproj(:,i));


                %***** BEST FROM GROUNDTRUTH
                [pose_best(:,i), err, idxSel_best(:,i)] = chooseMinPoseErr(pq_arr_quad(:,:), pose_true(:,i), 1.2, 1);
                pose_err(:,i) =err'; 
               
            end
        end
    end
    %% Compare types : get selection from groundtruth

    %% Compare types : use most inliers with tiebreaker and min reproj when needed
    agree_mostInliers = (idxSel_mostInliers == idxSel_best);
    for i=1:size(agree_mostInliers,2)
       if (agree_mostInliers(i)==0) && (~isnan(pose_best(1,i)))
           if pose_true(:,i) == pose_mostInliers(:,i)
               agree_mostInliers(:, i)=1;
           end
       end
    end
    mask = isnan(idxSel_best(1,:));
    agree_mostInliers_masked = agree_mostInliers;
    agree_mostInliers_masked(mask)=[];
    agreeFrac_mostInliers = sum(agree_mostInliers_masked)/size(agree_mostInliers_masked, 2);

     %% Compare types : min reproj 
    agree_minReproj = (idxSel_minReproj == idxSel_best);
    for i=1:size(agree_minReproj,2)
       if (agree_minReproj(i)==0) && (~isnan(pose_best(1,i)))
           if pose_true(:,i) == pose_minReproj(:,i)
               agree_minReproj(:, i)=1;
           end
       end
    end
    mask = isnan(idxSel_best(1,:));
    agree_minReproj_masked = agree_minReproj;
    agree_minReproj_masked(mask)=[];
    agreeFrac_minReproj = sum(agree_minReproj_masked)/size(agree_minReproj_masked, 2);

    
     %% Benchmark
    idx_validP3p = ~isnan(pose_best(1,:));
    trajErr = evaluateTrackingPerformance(pose_statusQuo(:,idx_validP3p), pose_true(:,idx_validP3p), 'none');
    figure;
    %plot(trajErr.AbsoluteError(:,2))
    plot(trajErr, "absolute-translation");
    figure;
    plot(trajErr, "absolute-rotation");



 