fileName="C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\sanityChecks\wiggle\ekfLog_wiggle.txt";

[ekfResult, groundTruth, trajErr]=readEkfLog(fileName, true);

figure;
plot(ekfResult.elapsedTime, ekfResult.u(4:6,:));
rawNorm = vecnorm(ekfResult.u(4:6,:),2,1);
hold on;
plot(ekfResult.elapsedTime, rawNorm);

figure;
plot(ekfResult.elapsedTime, ekfResult.x_(11:13,:));

debiasedU_live = ekfResult.u(4:6,:)-ekfResult.x_(11:13,:);
debiasedNorm_live=vecnorm(debiasedU_live,2,1);
figure;
plot(ekfResult.elapsedTime, debiasedNorm_live);
debiasedSmooth_live=smoothdata(debiasedNorm_live,2);
hold on;

convBias = [-0.29, -0.1,  0]';
debiasedU_conv = ekfResult.u(4:6,:)- repmat(convBias, [1,size(ekfResult.elapsedTime,2)]);
debiasedNorm_conv = vecnorm(debiasedU_conv,2,1);
debiasedSmooth_conv = smoothdata(debiasedNorm_conv,2);
plot(ekfResult.elapsedTime, debiasedSmooth_conv);


rawsmooth = smoothdata(rawNorm,2);
hold on;
plot(ekfResult.elapsedTime, rawsmooth);