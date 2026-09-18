

z_time = ekfResult.time(:,~isnan(ekfResult.z(1,:)));
z_timeEl = diff(z_time);
z_timeEl(:,1)=[];
z_rate = 1./(z_timeEl*10^-6);


rateVec = [stat1_z_rate, stat2_z_rate, mov1_z_rate, mov2_z_rate];
grpVec = [ones(size(stat1_z_rate)), 2.*ones(size(mov1_z_rate)), 3.*ones(size(mov2_z_rate))];
boxplot(rateVec, grpVec')


x_timeEl = diff(ekfResult.time);
x_timeEl(:,1)=[];
x_rate = 1./(x_timeEl*10^-6);

figure;
rateVec = [x_rate1, x_rate2, x_rate3];
grpVec = [ones(size(x_rate1)), 2.*ones(size(x_rate2)), 3.*ones(size(x_rate3))];
boxplot(rateVec, grpVec')