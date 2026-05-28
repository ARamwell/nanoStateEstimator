

z_time = ekfResult.time(:,~isnan(ekfResult.z(1,:)));
z_timeEl = diff(z_time);
z_timeEl(:,1)=[];
z_rate = 1./(z_timeEl*10^-6);


rateVec = [stat1_z_rate, stat2_z_rate, mov1_z_rate, mov2_z_rate];
grpVec = [ones(size(stat1_z_rate)), 2.*ones(size(mov1_z_rate)), 3.*ones(size(mov2_z_rate))];
boxplot(rateVec, grpVec')