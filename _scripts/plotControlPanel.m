% Control Panel plots only

figure('Name','Control Panel','NumberTitle','off')

% Switch Key Switch
subplot(5,3,1)
switchKeySwitch = bitUnpack(rt_ControlPanel,1,1);
plot(rt_tout,switchKeySwitch)
xlabel('Time (s)')
title('Key Switch')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Switch HV Enable
subplot(5,3,2)
switchHVEnable = bitUnpack(rt_ControlPanel,1,2);
plot(rt_tout,switchHVEnable)
xlabel('Time (s)')
title('HV Enable')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Switch DC/DC On
subplot(5,3,3)
switchDCDCOn = bitUnpack(rt_ControlPanel,1,3);
plot(rt_tout,switchDCDCOn)
xlabel('Time (s)')
title('DC/DC On')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Switch Utility 1
subplot(5,3,4)
switchUtility1 = bitUnpack(rt_ControlPanel,1,4);
plot(rt_tout,switchUtility1)
xlabel('Time (s)')
title('Utility 1')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Switch Utility 2
subplot(5,3,5)
switchUtility2 = bitUnpack(rt_ControlPanel,1,5);
plot(rt_tout,switchUtility2)
xlabel('Time (s)')
title('Utility 2')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Switch Utility 3
subplot(5,3,6)
switchUtility3 = bitUnpack(rt_ControlPanel,1,6);
plot(rt_tout,switchUtility3)
xlabel('Time (s)')
title('Utility 3')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Switch Cruise Set
subplot(5,3,7)
switchCruiseSet = bitUnpack(rt_ControlPanel,1,7);
plot(rt_tout,switchCruiseSet)
xlabel('Time (s)')
title('Cruise Set')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Switch Cruise Enable
subplot(5,3,8)
switchCruiseEnable = bitUnpack(rt_ControlPanel,1,8);
plot(rt_tout,switchCruiseEnable)
xlabel('Time (s)')
title('Cruise Enable')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Lamp FNR (F)
subplot(5,3,9)
lampFNRF = bitUnpack(rt_ControlPanel,2,1);
plot(rt_tout,lampFNRF)
xlabel('Time (s)')
title('Lamp FNR (F)')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Lamp FNR (R)
subplot(5,3,10)
lampFNRR = bitUnpack(rt_ControlPanel,2,2);
plot(rt_tout,lampFNRR)
xlabel('Time (s)')
title('Lamp FNR (R)')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Lamp GPS OK
subplot(5,3,11)
lampGPSOK = bitUnpack(rt_ControlPanel,2,3);
plot(rt_tout,lampGPSOK)
xlabel('Time (s)')
title('Lamp GPS OK')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Lamp WFT OK
subplot(5,3,12)
lampWFTOK = bitUnpack(rt_ControlPanel,2,4);
plot(rt_tout,lampWFTOK)
xlabel('Time (s)')
title('Lamp WFT OK')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Lamp Drive Fault
subplot(5,3,13)
lampDriveFault = bitUnpack(rt_ControlPanel,2,5);
plot(rt_tout,lampDriveFault)
xlabel('Time (s)')
title('Lamp Drive Fault')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Lamp Steer Fault
subplot(5,3,14)
lampSteerFault = bitUnpack(rt_ControlPanel,2,6);
plot(rt_tout,lampSteerFault)
xlabel('Time (s)')
title('Lamp Steer Fault')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])

% Lamp DC/DC OK
subplot(5,3,15)
lampDCDCOK = bitUnpack(rt_ControlPanel,2,7);
plot(rt_tout,lampDCDCOK)
xlabel('Time (s)')
title('Lamp DC/DC OK')
yticks([0 1])
yticklabels({'Off' 'On'})
ylim([-0.1 1.1])
