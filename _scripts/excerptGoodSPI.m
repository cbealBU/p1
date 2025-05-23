% Script to excerpt only the good data in the set, avoiding bad SPI
% transmissions

% DEPRECATED - this used basic checks to see if the data "looked good"
% based on patterns of zeros/255's, etc. The new approach has a data header
% and a data footer to check.
%spiGoodInds = find(sum(diff(double(rt_spiBytesIn),1,2)==0,2)<100);

% CEB: Vectorize this new approach later?
% Preallocate a vector of good data indices
spiGoodInds = nan(size(rt_tout));

% Set a counter for the number of indices of good data
goodIndCount = 0;
% Loop through each time step, determining whether there is good data based
% on the header and footer
for jj = 1:length(rt_tout)
    cond1 = all(rt_spiBytesIn(jj,1:10) == uint8([115 116 97 114 116 0 0 0 0 0]));
    cond2 = all(rt_spiBytesIn(jj,end-4:end) == uint8([0 0 101 110 100]));
    if cond1 && cond2
        goodIndCount = goodIndCount + 1;
        spiGoodInds(goodIndCount) = jj;
    end
end
% Trim the preallocated spiGoodInds vector to only the valid indices
spiGoodInds = spiGoodInds(1:goodIndCount);

% rt_adc0 = rt_adc0(spiGoodInds);
% rt_adc1 = rt_adc1(spiGoodInds);
% rt_adc2 = rt_adc2(spiGoodInds);
% rt_adc3 = rt_adc3(spiGoodInds);
% rt_adc4 = rt_adc4(spiGoodInds);
% rt_adc5 = rt_adc5(spiGoodInds);
% rt_adc6 = rt_adc6(spiGoodInds);
% rt_adc7 = rt_adc7(spiGoodInds);
rt_ControlPanel = rt_ControlPanel(spiGoodInds,:);
rt_DriverInput = rt_DriverInput(spiGoodInds,:);
rt_DrivetrainLeft = rt_DrivetrainLeft(spiGoodInds,:);
rt_DrivetrainRight = rt_DrivetrainRight(spiGoodInds,:);
rt_GPS = rt_GPS(spiGoodInds,:);
rt_Ignition = rt_Ignition(spiGoodInds,:);
rt_IMU = rt_IMU(spiGoodInds,:);
%rt_SPI_tout = rt_SPI_tout(spiGoodInds);
rt_spiBytesIn = rt_spiBytesIn(spiGoodInds,:);
rt_SteeringLeft = rt_SteeringLeft(spiGoodInds,:);
rt_SteeringRight = rt_SteeringRight(spiGoodInds,:);
rt_tout = rt_tout(spiGoodInds,:);
rt_WheelForceLeft = rt_WheelForceLeft(spiGoodInds,:);
rt_WheelForceRight = rt_WheelForceRight(spiGoodInds,:);
