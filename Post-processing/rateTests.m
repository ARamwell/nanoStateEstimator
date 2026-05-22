

stat2_z_time = stat2_ekfResult.time(:,~isnan(stat2_ekfResult.z(1,:)));
stat2_z_timeEl = diff(stat2_z_time);
stat2_z_timeEl(:,1)=[];
stat2_z_rate = 1./(stat2_z_timeEl*10^-6);
rateVec = [stat1_z_rate, stat2_z_rate, mov1_z_rate, mov2_z_rate];
grpVec = [ones(size(stat1_z_rate)), 2.*ones(size(mov1_z_rate)), 3.*ones(size(mov2_z_rate))];
boxplot(rateVec, grpVec')