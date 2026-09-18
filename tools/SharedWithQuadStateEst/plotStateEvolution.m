function fig_stateEvol = plotStateEvolution(ekfResult, trueState, trueState_times, p3p_sel, p3p_times)
    
    numLoops = size(ekfResult.x_, 2);
    numEl = size(ekfResult.x_, 1);
    
    fig_stateEvol = figure();
    if numEl == 10
        numVertTiles = 3;
    elseif numEl == 16
        numVertTiles = 5;
    end
    %tileLayout_cust = tiledlayout(fig_stateEvol, numVertTiles, 12);
    stateNames = {"$p_x$", "$p_y$", "$p_z$", "$q_w$", "$q_x$", "$q_y$", "$q_z$", "$v_x$", "$v_y$", "$v_z$", "$ba_x$", "$ba_y$", "$ba_z$", "$bg_x$", "$bg_y$","$bg_z$"};

    %turn groundtruth quaternion around if needed
    for t=1:size(trueState_times, 2)
        [~, idx]=min(abs(ekfResult.elapsedTime(1,:)-trueState_times(1,t)));
        if dot(ekfResult.x_(4:7, idx), trueState(4:7,t))<0
             trueState(4:7,t)=- trueState(4:7,t);
        end
    end
    
    for i=1:numEl
        
    
        %PLOT CHANGING STATE
        figObj=figure;
        % if i>=4 && i<=7
        %     nexttile([1 3]);
        % else
        %     nexttile([1 4]);
        % end
        x = ekfResult.elapsedTime(1,:);
        y = ekfResult.x_(i,:);

        plot(x, y, Color='#d95319',  DisplayName='estimated state variable',LineWidth=1.5);  
        hold on;

        stateVariance = zeros(1, numLoops); %initialise array
        for t=1:numLoops
            stateVariance(1,t)=(ekfResult.P(i,i,t));
        end
        stateVariance = sqrt(stateVariance);
        lowerCurve = y - 2*abs(stateVariance);
        upperCurve = y + 2*abs(stateVariance);
        plot(x, lowerCurve, Color='#77ac30',  HandleVisibility='off',LineWidth=1.5);
        hold on;
        plot(x, upperCurve, Color='#77ac30', DisplayName='2-std dev bounds', LineWidth=1.5);
        hold on;
        % fill(x, [y; upperCurve],[.9 .9 .9],'linestyle','none');
        % hold on;
        % 
        % fill(x, [lowerCurve; y],[.9 .9 .9],'linestyle','none');
        % hold on;
        %line(x,y)
        %errorbar(x, y, -dy, dy);
        % 
        % drawnow;
        % hold on;
    
    
        %%PLOT GROUND TRUTH
        if i <= 7
            if exist("p3p_sel", "var")
                x=p3p_times;
                y=p3p_sel(i,:);
                %x = p3pArr.time(1,:);
                %y = p3pArr.selected(i, :);
                plot(x, y, Color='#edb120',  DisplayName='visual state estimate',LineWidth=1.5);  
                hold on;
            end

        end

        if i<= 10
            if exist("trueState", 'var')
                %figure()
                x = trueState_times(1,:);
                y = trueState(i,:);
               
                plot(x, y, Color='#0072bd',  DisplayName='true state variable',LineWidth=1.5);
                hold on;
            end
        end

    
        %plot corrections, if any (i.e., measurements in state space)
        % if i<=7
        %     %plot(x(isfinite(y)),y(isfinite(y)),'*-')
        %     x = ekfResult.elapsedTime(1,:);
        %     y = ekfResult.z(i,:);
        %     scatter(x(isfinite(y)),y(isfinite(y)),10, "filled");
        %     hold on;
        %     plot(x,y);
        %     hold on;
        % 
        %     % %mark estimate that was corrected
        %     % correctedEst = zeros(1, size(ekfResult.zHist, 2));
        %     % for t=1:size(ekfResult.zHist, 2)
        %     %     t_k =  ekfResult.zHist(1, t);
        %     %     [closestDiff, closestIndex] = min(abs(ekfResult.elapsedTime(1,:)-ekfResult.zHist(1,t)));
        %     %     correctedEst(1, t) = ekfResult.stateEst(i, closestIndex);
        %     % end
        %     % y = correctedEst;
        %     % quiver(x, correctedEst, zeros(1, size(correctedEst, 2)), ((ekfResult.zHist(1+i,:)-correctedEst)), 0);
        % 
        % end
        
        ylabel(string(stateNames{i}));
        xlabel("elapsed time (seconds)");
        %legend('State estimate', 'Std deviation lower bound', 'Std deviation upper bound', 'Ground truth', 'Camera estimate (correction)')
        legend;
        %xlim([0,10]);
        hold off;

        

        formatFigForLatex(figObj);
        
    
       end
end