classdef imgFuncs
    methods (Static)

        %------------------------------------------------------------%
        function [rtHist, quadStateHist_times, quadStateHist, cam_timeHist, imuReadHist, imu_timeHist] = importSimLog(fullFile, refTime)
        %Basic function to import all logged trajectory data (as .m file). 
        %Does not consider time alignment with other data (i.e., imports 
        %all logged points, does not skip any)
            %refTime = datetime(2000, 01, 01);

            simData = load(fullFile);
            numQuadLogPoints = size((simData.out.quadState.signals.values), 3);
            quadStateDimensions = size((simData.out.quadState.signals.values), 2);

            numCamLogPoints = size((simData.out.camState_GT.signals.values), 3);
            
            numImuReadPoints = size(simData.out.IMU.signals.values, 3);

            %numImuLogPoints = size(simData.out.IMUstate.signals.values, 3);

            %Initialise output variables
            rtHist = zeros(3,4,numQuadLogPoints-1);%-1 to skip t0
            %quadStateHist_times = zeros(1, (numQuadLogPoints-1));
            quadStateHist =zeros(quadStateDimensions, (numQuadLogPoints-1));
            imuReadHist =zeros(6, (numImuReadPoints-1));
                        
            %imu_timeHist = datetime(mocapData.timestamps_mocap, 'Format', 'yyyyMMdd_HHmmss_SSS');createArray(zeros(1, (numImuReadPoints-1));
            %imu_timeHist = dateTime(2000, 01, 01)
            %imuStateHist = zeros(10+1, (numImuLogPoints-1));

             % %Import time history
             for i=1:size((simData.out.camState_GT.signals.values), 3)
                 cam_time = simData.out.camState_GT.time(i,1);
                 cam_timeHist(1,(i)) = datetime(refTime+seconds(cam_time), 'Format', 'yyyyMMdd_HHmmss_SSS');
                 %cam_timeHist(1,(i)) = cam_times;
             end

            %For each logged time - skip t0, where there is no data
            for i=1:numCamLogPoints

                %% Import Camera Ground Truth data
                % 
                % % Import position log
                % trans_cam = transpose(simData.out.camState_GT.signals.values(1,1:3,i));
                % trans_cam = trans_cam; %In m
                % 

                % 
                % %If in quaternions (new implementation)
                %     %import orientation log
                %     q = simData.out.camState_GT.signals.values(1, 4:7, i);
                %     j = 8;
                %     orient = transpose(q);
                % 
                %     %convert to rotation matrix
                %     R_cam = quat2rotm(q);
                % 
                % % Import velocity log
                % x_dot = simData.out.camState_GT.signals.values(1,j,i);
                % y_dot = simData.out.camState_GT.signals.values(1,j+1,i);
                % z_dot = simData.out.camState_GT.signals.values(1,j+2,i);
                % 
                % %Buld ground truth Rt history
                % rtHist(1:3,1:3,i) = R_cam;
                % rtHist(1:3,4,i) = trans_cam;
                % stateHist(:,i) = ([trans_cam; orient; x_dot; y_dot; z_dot]);
            end
            

                 %% Import Quad Ground Truth data
            for i=1:numQuadLogPoints
                 %Import time history
                state_time = simData.out.quadState.time(i,1);
                quadStateHist_times(1,(i)) = datetime(refTime+seconds(state_time), 'Format', 'yyyyMMdd_HHmmss_SSS');

                % Import position log
                trans_quad = transpose(simData.out.quadState.signals.values(1,1:3,i)); %in m
             
                %If in quaternions (new implementation)
                %import orientation log
                q = simData.out.quadState.signals.values(1, 4:7, i);
                j = 8;
                orient = transpose(q);

                %convert to rotation matrix
                R_quad =(quat2rotm(q));

                % Import velocity log
                x_dot = simData.out.quadState.signals.values(1,j,i);
                y_dot = simData.out.quadState.signals.values(1,j+1,i);
                z_dot = simData.out.quadState.signals.values(1,j+2,i);

                % Import bias log
                j=11;
                b_a = simData.out.quadState.signals.values(1,j:j+2,i);
                b_g = simData.out.quadState.signals.values(1,j+3:j+5,i);
                
                %Buld ground truth Rt history
                rtHist(1:3,1:3,i) = R_quad;
                rtHist(1:3,4,i) = trans_quad;
                quadStateHist(:,i) = ([trans_quad; orient; x_dot; y_dot; z_dot; b_a'; b_g']);
            end
            
            
                               
            %% Import IMU readings

            for i=1:numImuReadPoints
                % Import imu log
                imuState = simData.out.IMU.signals.values(:,:,i);
                imuState_time = simData.out.IMU.time(i,1);
                %build Rt history
                imu_timeHist(1,i)= datetime(refTime+seconds(imuState_time), 'Format', 'yyyyMMdd_HHmmss_SSS');
                imuReadHist(1:end, i)=transpose(imuState); 

            end

            % %% Import IMU log data
            % 
            % for i=1:numImuLogPoints
            %     % Import imu log
            %     imuState = simData.out.IMUstate.signals.values(:,:,i);
            %     imuState_time = simData.out.IMUstate.time(i,1);
            % 
            % 
            %     %build Rt history
            %     imuStateHist(1,i)=imuState_time;
            %     imuReadHist(2:end, i)=transpose(imuState); 
            % 
            % end
        end


        %------------------------------------------------------------% 

        function convertVideo(sourceFile, targetFolder)
            %Convert video into series of frames. Name the frames according
            %to the time elapsed from video start. 
            
            %read video
            vid = VideoReader(sourceFile);

            %determine video parameters
            numFrames = vid.NumFrames;
            vidDuration = vid.Duration;
            fps = numFrames/vidDuration;
            secPerFrame = 1/fps;

            %frame-by-frame, convert frames to images and save
            for f=1:numFrames
                frame = read(vid, f); %read frame
                t=(f-1)*secPerFrame*1000; %get time in ms
                t = num2str(t);
                %frame_g = rgb2gray(frame_rgb); %convert to grayscale
                
                %save image to target folder
                while strlength(t)<6
                    t=strcat("0",t);
                end
              
                outputBaseFileName=strcat(t, ".jpg");
                %outputBaseFileName = sprintf('-%4.4d.jpg', t);
                outputFullFileName = fullfile(targetFolder, outputBaseFileName); %output filename
                imwrite(frame, outputFullFileName, 'jpg'); %write output file
            end

        end
        %------------------------------------------------------------%
            

        function [I_seq, I_seq_t] = importImageSeq(imgFolder, realTimestamps, refTime, ext)
            %Function to import images from a folder into a big 3D array.
            %Also outputs the time (in ms), from the video start, to the
            %respective frame. 

            imgFiles_ds = fileDatastore(imgFolder, 'ReadFcn', @importdata, "FileExtensions",ext);
            imgNames = imgFiles_ds.Files;
            numImgs = length(imgNames);

            %Define output variables
            I_seq = zeros(1,1,1);
            %I_seq_t = createArray(1,numImgs, "datetime");

            %image-by-image
            for f=1:numImgs
                I_rgb = imread(string(imgNames(f))); %read image into array
                I_g = rgb2gray(I_rgb);%convert to grayscale
                if f==1
                    I_seq = I_g;
                else
                    I_seq(:,:,f)=I_g; %Put into big array
                end
                
                %Get time
                [I_filepath, I_name, I_ext] = fileparts(imgNames(f)); %separate name portion

                %if real time stamps
                if realTimestamps == 1
                    tokens = regexp(I_name, '_(\d{8}_\d{6}_\d{3})', 'tokens'); %extract timestamp from name
                    timestampStr = tokens{1}{1};  % e.g. '20250505_132513_986'
                    I_seq_t(f) =  datetime(timestampStr, 'InputFormat', 'yyyyMMdd_HHmmss_SSS', 'Format','yyyyMMdd_HHmmss_SSS');
                    %I_seq_t(f) = dt;
                else %if from simulation
                    I_seq_t(f) = datetime(refTime+seconds(str2double(I_name)/1000), 'Format', 'yyyyMMdd_HHmmss_SSS');
                end
            end
        end

        %------------------------------------------------------------%

        function I_annotated = markDetectedCheckers(I, x_pnts_i)
            %Add dots to show detected corners. Also label top-left point.

            I_annotated = I;
            
            %Mark corners with circles
            I_annotated = insertMarker(I_annotated, transpose(x_pnts_i(:,2:end)), "o");

            %Except for top left corner, which is marked with a square
            I_annotated = insertMarker(I_annotated, transpose(x_pnts_i(:,1)), "s");

        end



    end
end
