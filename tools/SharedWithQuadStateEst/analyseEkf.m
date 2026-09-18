function [ekfMetrics, areFig, ateFig, neesFig, nisFig] = analyseEkf(ekfSize,ekfName,trajFolderList, extraID, doScrub)
%ANALYSEEKF Summary of this function goes here
%   Detailed explanation goes here
    doPlot = true;

    %% do some analysis for this type of EKF
    trajErrArr = createArray(2,0);
    simpleErrArr = createArray(ekfSize,0);
    percentVioArr = createArray(ekfSize,0);
    neesPercArr =  createArray(ekfSize-1,0);
    nisPercArr = createArray(6,0);
    ttcArr = createArray(1,0);
    ttcNeesArr = createArray(1,0);
    maxStableErrArr =createArray(5,0);
    trajNameArr = {};

    gtTimeArr=createArray(1,0);
    timeArr = createArray(1,0);
    biasArr = createArray(6,0);
    residArr = createArray(7,0);
    PArr=createArray(ekfSize,0);

    if ~doPlot
        areFig=0;
        neesFig=0;
        ateFig=0;
        nisFig=0;
    end
      %% init figures
      if doPlot
            
            areFig = figure();
            hold on;
            title(strcat("ARE: ", string(ekfName), extraID));
            xlabel("time (s)");
            ylabel("absolute rotation error (degrees)");
        
            ateFig = figure();
            hold on;
            title(strcat("ATE: ", string(ekfName), extraID));
            xlabel("time (s)");
            ylabel("absolute translation error (m)");
        
            neesFig = figure();
            hold on;
            title(strcat("NEES: ", string(ekfName), extraID));
            xlabel("time (s)");
            ylabel("NEES")
        
            nisFig = figure();
            hold on;
            title(strcat("NIS: ", string(ekfName),extraID));
            xlabel("time (s)");
            ylabel("NIS")
      end

 %% Analysis

    for j=1:length(trajFolderList)
        ekfResult = load(trajFolderList{j});
        if size(fields(ekfResult),1)==1
            ekfResult = ekfResult.ekfResult;
        end
        parts = strsplit(string(trajFolderList{j}), '\');
        trajName = string((parts{end-1}));
        trajName = erase(trajName, "sim_");
        
        %*******************************************
        %Do some post-corrections:
       %  ekfResult.trueState(11:13,:)=-ekfResult.trueState(11:13,:); %ONLY FOR SIM! Accel bias is wrong way!
       %  ekfResult.z(:,1)=nan(size(ekfResult.z, 1),1);
       %  ekfResult.y(:,1)=nan(size(ekfResult.y, 1),1);
       %  ekfResult.nees = evalNEES_noq(ekfResult.x_, ekfResult.P, ekfResult.trueState(1:ekfSize,:));
       %  ekfResult.nis = evalNIS_noq(ekfResult.y, ekfResult.S);
       ekfResult.nees = evalNEES_noq_nano(ekfResult.x_, ekfResult.P, ekfResult.trueState(1:7,:));

        %***************************
                
        log_gtNotNan=~isnan(ekfResult.trueState(1,:));
        idx_gtNotNan = find(log_gtNotNan==1, sum(log_gtNotNan),"first");
        time_gt = ekfResult.time(1, log_gtNotNan);
        timeEl_gt = ekfResult.elapsedTime(1, log_gtNotNan);

        log_visNotNan=~isnan(ekfResult.z(1,:)) & sum(ekfResult.z,1)~=0;
        idx_visNotNan = find(log_visNotNan==1, sum(log_visNotNan), "first");
        time_vis = ekfResult.time(1, log_visNotNan);
        timeEl_vis = ekfResult.elapsedTime(1, log_visNotNan);

        %optionally, scrub of results with more than 1 second of blindndess
        if ~doScrub
            idx_scrubEnd = size(ekfResult.x_, 2);
            idx_scrubStart=1;
            idx_scrubStart_gt = 1;
            idx_scrubEnd_gt = sum(log_gtNotNan);
            idx_scrubStart_vis = 1;
            idx_scrubEnd_vis = sum(log_visNotNan);

        else %if doScrub
            % if median(ekfResult.timeSinceLastCorrection(2:end)) >1000 %check if actually in microseconds
            %     ekfResult.timeSinceLastCorrection =ekfResult.timeSinceLastCorrection *10^(-6);
            % end
            idx_scrubStart = find(ekfResult.timeSinceLastCorrection < 0.5, 1, "first");
            idx_scrubEnd = find(ekfResult.timeSinceLastCorrection(:, idx_scrubStart:end) > 1, 1, "first");
            idx_scrubEnd = idx_scrubStart+idx_scrubEnd;
            if ~exist('idx_scrubEnd', 'var') || isempty(idx_scrubEnd)
                idx_scrubEnd = size(ekfResult.x_, 2);
            end
            if ~exist('idx_scrubStart', 'var') || isempty(idx_scrubStart)
                idx_scrubStart=1;
            end

            %correlate with visual and ground truth indices      
            idx_scrubStart_gt = find(idx_gtNotNan>=idx_scrubStart, 1, "first");
            idx_scrubEnd_gt = find(idx_gtNotNan<=idx_scrubEnd, 1, "last");
            if ~exist('idx_scrubEnd_gt', 'var') || isempty(idx_scrubEnd_gt)
                idx_scrubEnd_gt = size(idx_gtNotNan,2);
            end
            if ~exist('idx_scrubStart_gt', 'var') || isempty(idx_scrubStart_gt)
                idx_scrubStart_gt=1;
            end
            %idx_scrubEnd_gt = idx_scrubEnd_gt-1;
    
            idx_scrubStart_vis = find(idx_visNotNan>=idx_scrubStart, 1, "first");
            idx_scrubEnd_vis = find(idx_visNotNan<=idx_scrubEnd, 1, "last");
                        if ~exist('idx_scrubEnd_vis', 'var') || isempty(idx_scrubEnd_vis)
                idx_scrubEnd_gt = size(idx_visNotNan,2);
            end
            if ~exist('idx_scrubStart_vis', 'var') || isempty(idx_scrubStart_vis)
                idx_scrubStart_vis=1;
            end
            %idx_scrubEnd_vis = idx_scrubEnd_vis-1;

        end

        idx_gt = sum(log_gtNotNan(1:idx_scrubEnd));

        ekfSelector = [ones(1, idx_scrubStart-1), zeros(1, idx_scrubEnd-idx_scrubStart+1), ones(1,size(ekfResult.x_,2)-idx_scrubEnd)];
        ekfSelector = logical(ekfSelector);
        ekfSelector = ~ekfSelector;
        ekfSelector = ekfSelector & log_gtNotNan;

       
        %*******************************************
        %time to converge
        [ttc, idxTtc, ~]=calcTimeToConverge(ekfResult.elapsedTime(ekfSelector), ekfResult.x_(:,ekfSelector), ekfResult.trueState(:,ekfSelector), 0.3,5, 1);

        % ttc_are_idx = find(ekfResult.trajErr.AbsoluteError(idx_scrubStart_gt:idx_scrubEnd_gt,1)'<= 5, 1, "first");%time to converge
        % if isempty(ttc_are_idx)
        %     ttc_are = nan(1);
        % else
        %     %tempIdx = find(gtNotNan==1,ttc_are_idx,'first');
        %     %ttc_are_idx = tempIdx(end);
        %     %ttc_are = ekfResult.time(1, ttc_are_idx);
        %     ttc_are = timeEl_gt(1, ttc_are_idx);
        % end
        % 
        % ttc_ate_idx = find(ekfResult.trajErr.AbsoluteError(idx_scrubStart_gt:idx_scrubEnd_gt,2)'<= 0.3, 1, "first");
        % if isempty(ttc_ate_idx)
        %     ttc_ate = nan(1,1);
        % else
        %     % tempIdx = find(gtNotNan==1,ttc_ate_idx,'first');
        %     % ttc_ate_idx = tempIdx(end);
        %     % ttc_ate = ekfResult.time(1, ttc_ate_idx);
        %     ttc_ate = timeEl_gt(1, ttc_ate_idx);
        % end
        % ttc=[ttc_are; ttc_ate];
        % 
        %
        validNees = ekfResult.nees(:, ekfSelector);
        % ttc_nees = ekfResult.time(:,testNeesConv(validNees));

           
        % %remove  first 20 estimates
        % ekfMetrics = ekfMetricsArr(i);
        % [trajErrClean, rmIdx] = rmoutliers(ekfMetrics.trajErr(:,20:end)', "mean");
        % ekfMetrics.trajErr_clean = trajErrClean';
        % ekfMetrics.simpleErr_clean
       
        %*******************************************
        %simple error 
        %ekfResult.simpleErr = ekfResult.trueState-ekfResult.x_(size(ekfResult.trueState, 1),:);

        %*******************************************
        %nees percent exceed
        alpha = 0.05; %confidence
        numMonteCarloRuns = 1;%;length(selectListOfFileNames);
        stateSize = ekfSize-1;
        chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*stateSize)]/numMonteCarloRuns;
        nees_i =ekfResult.nees(:, idx_scrubStart:idx_scrubEnd);
        neesVioHi_idx = find(nees_i > chiSquareLimits(2), size(nees_i,2));
        neesVioLo_idx =find(nees_i < chiSquareLimits(1), size(nees_i,2));
        neesVioEith_idx = unique([neesVioHi_idx, neesVioLo_idx]);
        neesVioPercent = [size(neesVioLo_idx, 2); size(neesVioHi_idx, 2); size(neesVioEith_idx, 2)]/(idx_scrubEnd_gt-idx_scrubStart_gt);
        nees_i = rmoutliers(nees_i, 'mean', 'ThresholdFactor', 8);
        
        
        %*******************************************
        %nis percent exceed
        alpha = 0.05; %confidence
        numMonteCarloRuns = 1;%length(selectListOfFileNames);
        measSize = 6;
        
        validNis = ekfResult.nis(:,idx_scrubStart:idx_scrubEnd);

        % if size(ekfResult.nis, 2)< size(ekfResult.x_, 2)
        %     nis_idx = idx_scrubEnd_vis;%size(ekfResult.nis, 2);
        % else
        %     nis_idx =idx_scrubEnd;
        % end
        nis_i = validNis;
        chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*measSize)]/numMonteCarloRuns;
        nisVioHi_idx = find(nis_i > chiSquareLimits(2), size(nis_i,2));
        nisVioLo_idx =find(nis_i < chiSquareLimits(1), size(nis_i,2));
        nisVioEith_idx = unique([nisVioHi_idx, nisVioLo_idx]);
        nisVioPercent = [size(nisVioLo_idx, 2); size(nisVioHi_idx, 2); size(nisVioEith_idx, 2)]/(idx_scrubEnd_vis-idx_scrubStart_vis);
        nan_idx_nis = ~isnan(nis_i);
        %nis_i = nis_i(:,nan_idx_nis);
        nis_t = ekfResult.elapsedTime(:,idx_scrubStart:idx_scrubEnd);
        %nis_i =rmoutliers(nis_i, 'mean', 'ThresholdFactor', 8);
        
    
        %*******************************************
        % 
        % %max, min, mean error per state - as much as possible
        gtSize = size(ekfResult.trueState, 1);
        gtTimes = ekfResult.elapsedTime(1, ekfSelector);
        if ekfSize<gtSize
            gtSize=ekfSize;
        end
        trajErr_i =ekfResult.trajErr.AbsoluteError(idx_scrubStart_gt:idx_scrubEnd_gt, :)';

        % get percent divergence per state
        if isfield(ekfResult, 'P')
            [vio, x_err]=evalPercentDivergence(ekfResult.x_(1:gtSize, ekfSelector), ekfResult.trueState(1:gtSize, ekfSelector), ekfResult.P(1:gtSize, 1:gtSize,ekfSelector), 2);
        else
            [vio, x_err]=evalPercentDivergence(ekfResult.x_(1:gtSize,ekfSelector), ekfResult.trueState(1:gtSize, ekfSelector), ekfResult.PHat(1:gtSize, 1:gtSize,ekfSelector, 2));
        end
        if gtSize > 7
            velErr_i = abs(vecnorm(x_err(8:10,:), 2, 1));
            velErr_i = rmoutliers(velErr_i, 'mean');
            if size(x_err,1)>10
                baErr_i = abs(vecnorm(x_err(11:13,:), 2, 1));
                bgErr_i = abs(vecnorm(x_err(14:16,:), 2, 1));
            else
                baErr_i = nan(size(x_err, 2));
                bgErr_i= nan(size(x_err, 2));
            end
        end

        %*******************************************
        % get max stabilised error per state
        if ~isnan(idxTtc)
            maxStableErr(1,1) = max(trajErr_i(2,idxTtc:end));
            maxStableErr(2,1) = max(trajErr_i(1,idxTtc:end));
            if gtSize>7
                maxStableErr(3,1) = max(velErr_i(idxTtc:end));
            %if size(ekfMetricsArr(i).simpleErr, 1) > 10
                maxStableErr(4,1) = max(baErr_i(idxTtc:end));
                maxStableErr(5,1) = max(bgErr_i(idxTtc:end));
            %end
            end
        end

        % % put all vel, all bias together? like position
        % velErr = vecnorm(x_err(8:10,:), 2,2);
        % if stateSize>10
        %     baErr = vecnorm()
           % Get a few extra error metrics
           % if ekfSize>10
           %      selTime = ekfResult.elapsedTime(1,ekfSelector);
           %      i5 = find(selTime(1,:)>=5, 1, "first");
           %      i10 = find(selTime(1,:)>=10, 1, "first");
           %      i20 = find(selTime(1,:)>=20, 1, "first");
           %      i30 = find(selTime(1,:)>=30, 1, "first");
           %      i40 = find(selTime(1,:)>=40, 1, "first");
           %      i60 = find(selTime(1,:)>=60, 1, "first");
           %      i80 = find(selTime(1,:)>=80, 1, "first");
           %      i100 = find(selTime(1,:)>=99, 1, "first");
           %      idxSel = [i5, i10, i20, i30, i40, i60, i80, i100];
           %      (x_err(11:16,idxSel));
           % end
           % 

           P_diag = createArray(size(ekfResult.P,1), size(ekfResult.P,3));
           P_var = P_diag;
           for t=1:size(P_diag,2)
               P_diag(:,t)=diag(ekfResult.P(:,:,t));
               P_var(:,t) =sqrt(P_diag(:,t));
           end
    %*******************************************
    %build arrays
        trajErrArr = [trajErrArr, trajErr_i];
        simpleErrArr = [simpleErrArr, x_err(:,500:end)];
        percentVioArr = [percentVioArr, vio];
        ttcArr = [ttcArr, ttc];
        nisPercArr = [nisPercArr, nisVioPercent];
        neesPercArr = [neesPercArr, neesVioPercent];
%        ttcNeesArr = [ttcNeesArr, ttc_nees];
        trajNameArr = [trajNameArr, trajName];
        gtTimeArr = [gtTimeArr, gtTimes];
        maxStableErrArr = [maxStableErrArr, maxStableErr];
        if ekfSize>10
            biasArr = [biasArr, ekfResult.x_(11:16,:)];
        end
        timeArr = [timeArr, ekfResult.elapsedTime];
        residArr = [residArr, ekfResult.y];
        PArr=[PArr, P_var(:,500:end)];
        % 

        %*******************************************
        %plot
        if doPlot
            figure(areFig);
            plot(timeEl_gt(1,idx_scrubStart_gt:idx_scrubEnd_gt),  ekfResult.trajErr.AbsoluteError(idx_scrubStart_gt:idx_scrubEnd_gt,1)', 'DisplayName', trajName);
            hold on;
    
            figure(ateFig);
            plot(timeEl_gt(1,idx_scrubStart_gt:idx_scrubEnd_gt),  ekfResult.trajErr.AbsoluteError(idx_scrubStart_gt:idx_scrubEnd_gt,2)','DisplayName', trajName);
            hold on;
            
            figure(neesFig);
            plot(ekfResult.elapsedTime(1, ekfSelector), validNees','DisplayName', trajName);
            hold on;
    
            figure(nisFig);
            plot(timeEl_vis(:,idx_scrubStart_vis:idx_scrubEnd_vis)',  nis_i(:, nan_idx_nis)','DisplayName', trajName);
            hold on;
        end
    end

    ekfMetrics.trajErr = trajErrArr;
    ekfMetrics.simpleErr = simpleErrArr;
    ekfMetrics.percentVio = percentVioArr;
    ekfMetrics.percentNees = neesPercArr;
    ekfMetrics.percentNis = nisPercArr;
    ekfMetrics.ttc = ttcArr;
    ekfMetrics.ekfType = ekfName;
    ekfMetrics.maxStableErr = maxStableErrArr;
    ekfMetrics.trajNames = trajNameArr;

    ekfMetrics.gtTimes = gtTimeArr;
    ekfMetrics.biasArr = biasArr;
    ekfMetrics.timeArr =timeArr;
    ekfMetrics.residArr=residArr;
    ekfMetrics.PArr=PArr;



function [idx_conv] = testNeesConv(nees)
    %get rolling window average
    nees_avg = movmean(nees, 10);

    %check convergence
    converged = false;
    conv_cnt = 0;
    conv_prev = false;
    dnees = createArray(1,size(nees,2));
    dnees(1) = 0;
    
    %we consider that the filter has converged if nees changes by less than 5%
    for i=2:size(nees_avg,2)
        dnees(i) = (norm(nees_avg(i)-nees_avg(i-1)))/norm(nees_avg(i-1));
        conv_i =false;       
        if dnees(i) < 0.01
            conv_i = true;
            if conv_prev == true
                conv_cnt = conv_cnt+1;
            else
                conv_cnt = 1;
            end
        end
        if conv_cnt ==1
            idx_conv = i;
        end
        if conv_cnt>=500
            converged = true;
        end
        if converged
            break;
        end
        conv_prev = conv_i;
    end
    if ~converged
        idx_conv = [];
    end
end



end