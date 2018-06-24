function bio=p1bio
bio = [];
bio(1).blkName='Data Logging/Named Mux';
bio(1).sigName='';
bio(1).portIdx=0;
bio(1).dim=[145,1];
bio(1).sigWidth=145;
bio(1).sigAddress='&p1_B.NamedMux[0]';
bio(1).ndims=2;
bio(1).size=[];

bio(getlenBIO) = bio(1);

bio(2).blkName='Controllers and Output/Steering Controller/Product';
bio(2).sigName='';
bio(2).portIdx=0;
bio(2).dim=[2,1];
bio(2).sigWidth=2;
bio(2).sigAddress='&p1_B.Product_b[0]';
bio(2).ndims=2;
bio(2).size=[];


bio(3).blkName='Controllers and Output/UI Controller/Heartbeat Signal';
bio(3).sigName='';
bio(3).portIdx=0;
bio(3).dim=[1,1];
bio(3).sigWidth=1;
bio(3).sigAddress='&p1_B.HeartbeatSignal';
bio(3).ndims=2;
bio(3).size=[];


bio(4).blkName='Hardware I//O/Input/Load Cell Scaling';
bio(4).sigName='Front Load Cells';
bio(4).portIdx=0;
bio(4).dim=[2,1];
bio(4).sigWidth=2;
bio(4).sigAddress='&p1_B.FrontLoadCells[0]';
bio(4).ndims=2;
bio(4).size=[];


bio(5).blkName='Hardware I//O/Input/Load Cell Scaling Rear';
bio(5).sigName='Rear Load Cells';
bio(5).portIdx=0;
bio(5).dim=[2,1];
bio(5).sigWidth=2;
bio(5).sigAddress='&p1_B.RearLoadCells[0]';
bio(5).ndims=2;
bio(5).size=[];


bio(6).blkName='Hardware I//O/Input/encoder scaling';
bio(6).sigName='Angles';
bio(6).portIdx=0;
bio(6).dim=[2,1];
bio(6).sigWidth=2;
bio(6).sigAddress='&p1_B.Angles[0]';
bio(6).ndims=2;
bio(6).size=[];


bio(7).blkName='Hardware I//O/Input/voltage scaling';
bio(7).sigName='Voltages';
bio(7).portIdx=0;
bio(7).dim=[2,1];
bio(7).sigWidth=2;
bio(7).sigAddress='&p1_B.Voltages[0]';
bio(7).ndims=2;
bio(7).size=[];


bio(8).blkName='Hardware I//O/Input/Absolute Encoder';
bio(8).sigName='';
bio(8).portIdx=0;
bio(8).dim=[1,1];
bio(8).sigWidth=1;
bio(8).sigAddress='&p1_B.AbsoluteEncoder';
bio(8).ndims=2;
bio(8).size=[];


bio(9).blkName='Hardware I//O/Input/Analog Input (DAS)';
bio(9).sigName='';
bio(9).portIdx=0;
bio(9).dim=[16,1];
bio(9).sigWidth=16;
bio(9).sigAddress='&p1_B.AnalogInputDAS[0]';
bio(9).ndims=2;
bio(9).size=[];


bio(10).blkName='Hardware I//O/Input/Analog Input (VSBC)';
bio(10).sigName='';
bio(10).portIdx=0;
bio(10).dim=[8,1];
bio(10).sigWidth=8;
bio(10).sigAddress='&p1_B.AnalogInputVSBC[0]';
bio(10).ndims=2;
bio(10).size=[];


bio(11).blkName='Hardware I//O/Input/Ruby-MM/p1';
bio(11).sigName='Fault Switch';
bio(11).portIdx=0;
bio(11).dim=[1,1];
bio(11).sigWidth=1;
bio(11).sigAddress='&p1_B.FaultSwitch';
bio(11).ndims=2;
bio(11).size=[];


bio(12).blkName='Hardware I//O/Input/Ruby-MM/p2';
bio(12).sigName='Auto-Steer Switch';
bio(12).portIdx=1;
bio(12).dim=[1,1];
bio(12).sigWidth=1;
bio(12).sigAddress='&p1_B.AutoSteerSwitch';
bio(12).ndims=2;
bio(12).size=[];


bio(13).blkName='Hardware I//O/Input/Ruby-MM/p3';
bio(13).sigName='';
bio(13).portIdx=2;
bio(13).dim=[1,1];
bio(13).sigWidth=1;
bio(13).sigAddress='&p1_B.RubyMM_o3';
bio(13).ndims=2;
bio(13).size=[];


bio(14).blkName='Hardware I//O/Input/Ruby-MM/p4';
bio(14).sigName='';
bio(14).portIdx=3;
bio(14).dim=[1,1];
bio(14).sigWidth=1;
bio(14).sigAddress='&p1_B.RubyMM_o4';
bio(14).ndims=2;
bio(14).size=[];


bio(15).blkName='Hardware I//O/Input/Wheelspeed Sensors';
bio(15).sigName='Front Wheel Speeds';
bio(15).portIdx=0;
bio(15).dim=[2,1];
bio(15).sigWidth=2;
bio(15).sigAddress='&p1_B.FrontWheelSpeeds[0]';
bio(15).ndims=2;
bio(15).size=[];


bio(16).blkName='Hardware I//O/Output/amps to volts';
bio(16).sigName='';
bio(16).portIdx=0;
bio(16).dim=[1,1];
bio(16).sigWidth=1;
bio(16).sigAddress='&p1_ConstB.ampstovolts';
bio(16).ndims=2;
bio(16).size=[];


bio(17).blkName='Hardware I//O/Output/voltage gain';
bio(17).sigName='';
bio(17).portIdx=0;
bio(17).dim=[4,1];
bio(17).sigWidth=4;
bio(17).sigAddress='&p1_B.voltagegain[0]';
bio(17).ndims=2;
bio(17).size=[];


bio(18).blkName='Hardware I//O/Output/saturation';
bio(18).sigName='';
bio(18).portIdx=0;
bio(18).dim=[1,1];
bio(18).sigWidth=1;
bio(18).sigAddress='&p1_B.saturation';
bio(18).ndims=2;
bio(18).size=[];


bio(19).blkName='Hardware I//O/Output/saturation 2';
bio(19).sigName='';
bio(19).portIdx=0;
bio(19).dim=[2,1];
bio(19).sigWidth=2;
bio(19).sigAddress='&p1_B.saturation2[0]';
bio(19).ndims=2;
bio(19).size=[];


bio(20).blkName='Sensor Data and Estimation/Nonlinear Observer/1//Fnf';
bio(20).sigName='muhat';
bio(20).portIdx=0;
bio(20).dim=[1,1];
bio(20).sigWidth=1;
bio(20).sigAddress='&p1_B.muhat';
bio(20).ndims=2;
bio(20).size=[];


bio(21).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p1';
bio(21).sigName='ahats';
bio(21).portIdx=0;
bio(21).dim=[2,1];
bio(21).sigWidth=2;
bio(21).sigAddress='&p1_B.ahats[0]';
bio(21).ndims=2;
bio(21).size=[];


bio(22).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p2';
bio(22).sigName='peakFyf';
bio(22).portIdx=1;
bio(22).dim=[1,1];
bio(22).sigWidth=1;
bio(22).sigAddress='&p1_B.peakFyf';
bio(22).ndims=2;
bio(22).size=[];


bio(23).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p3';
bio(23).sigName='estimation_flags';
bio(23).portIdx=2;
bio(23).dim=[5,1];
bio(23).sigWidth=5;
bio(23).sigAddress='&p1_B.estimation_flags[0]';
bio(23).ndims=2;
bio(23).size=[];


bio(24).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p4';
bio(24).sigName='pneumatic_trail';
bio(24).portIdx=3;
bio(24).dim=[2,1];
bio(24).sigWidth=2;
bio(24).sigAddress='&p1_B.pneumatic_trail[0]';
bio(24).ndims=2;
bio(24).size=[];


bio(25).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p5';
bio(25).sigName='peak_forces';
bio(25).portIdx=4;
bio(25).dim=[2,1];
bio(25).sigWidth=2;
bio(25).sigAddress='&p1_B.peak_forces[0]';
bio(25).ndims=2;
bio(25).size=[];


bio(26).blkName='Sensor Data and Estimation/Steering Kinematics/Composite Steer Angle Calculation';
bio(26).sigName='Composite Steer Angle';
bio(26).portIdx=0;
bio(26).dim=[1,1];
bio(26).sigWidth=1;
bio(26).sigAddress='&p1_B.CompositeSteerAngle';
bio(26).ndims=2;
bio(26).size=[];


bio(27).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain1';
bio(27).sigName='Speed';
bio(27).portIdx=0;
bio(27).dim=[1,1];
bio(27).sigWidth=1;
bio(27).sigAddress='&p1_B.Speed';
bio(27).ndims=2;
bio(27).size=[];


bio(28).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain2';
bio(28).sigName='';
bio(28).portIdx=0;
bio(28).dim=[1,1];
bio(28).sigWidth=1;
bio(28).sigAddress='&p1_B.Gain2';
bio(28).ndims=2;
bio(28).size=[];


bio(29).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain3';
bio(29).sigName='Speed';
bio(29).portIdx=0;
bio(29).dim=[1,1];
bio(29).sigWidth=1;
bio(29).sigAddress='&p1_B.Speed_b';
bio(29).ndims=2;
bio(29).size=[];


bio(30).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain4';
bio(30).sigName='';
bio(30).portIdx=0;
bio(30).dim=[1,1];
bio(30).sigWidth=1;
bio(30).sigAddress='&p1_B.Gain4';
bio(30).ndims=2;
bio(30).size=[];


bio(31).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain5';
bio(31).sigName='';
bio(31).portIdx=0;
bio(31).dim=[1,1];
bio(31).sigWidth=1;
bio(31).sigAddress='&p1_B.Gain5';
bio(31).ndims=2;
bio(31).size=[];


bio(32).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter';
bio(32).sigName='Front Left';
bio(32).portIdx=0;
bio(32).dim=[1,1];
bio(32).sigWidth=1;
bio(32).sigAddress='&p1_B.FrontLeft';
bio(32).ndims=2;
bio(32).size=[];


bio(33).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter1';
bio(33).sigName='Horiz Speed';
bio(33).portIdx=0;
bio(33).dim=[1,1];
bio(33).sigWidth=1;
bio(33).sigAddress='&p1_B.HorizSpeed';
bio(33).ndims=2;
bio(33).size=[];


bio(34).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter2';
bio(34).sigName='Front Right';
bio(34).portIdx=0;
bio(34).dim=[1,1];
bio(34).sigWidth=1;
bio(34).sigAddress='&p1_B.FrontRight';
bio(34).ndims=2;
bio(34).size=[];


bio(35).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter3';
bio(35).sigName='Rear Left';
bio(35).portIdx=0;
bio(35).dim=[1,1];
bio(35).sigWidth=1;
bio(35).sigAddress='&p1_B.RearLeft';
bio(35).ndims=2;
bio(35).size=[];


bio(36).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter4';
bio(36).sigName='Rear Right';
bio(36).portIdx=0;
bio(36).dim=[1,1];
bio(36).sigWidth=1;
bio(36).sigAddress='&p1_B.RearRight';
bio(36).ndims=2;
bio(36).size=[];


bio(37).blkName='User Controllers/Cruise Control/Sum1';
bio(37).sigName='';
bio(37).portIdx=0;
bio(37).dim=[1,1];
bio(37).sigWidth=1;
bio(37).sigAddress='&p1_B.Sum1_i';
bio(37).ndims=2;
bio(37).size=[];


bio(38).blkName='User Controllers/Cruise Control/Switch';
bio(38).sigName='';
bio(38).portIdx=0;
bio(38).dim=[1,1];
bio(38).sigWidth=1;
bio(38).sigAddress='&p1_B.Switch_o';
bio(38).ndims=2;
bio(38).size=[];


bio(39).blkName='User Controllers/Steering Controller/Heavy Filter';
bio(39).sigName='';
bio(39).portIdx=0;
bio(39).dim=[1,1];
bio(39).sigWidth=1;
bio(39).sigAddress='&p1_B.HeavyFilter';
bio(39).ndims=2;
bio(39).size=[];


bio(40).blkName='User Controllers/Steering Controller/Nominal Filter';
bio(40).sigName='';
bio(40).portIdx=0;
bio(40).dim=[1,1];
bio(40).sigWidth=1;
bio(40).sigAddress='&p1_B.NominalFilter';
bio(40).ndims=2;
bio(40).size=[];


bio(41).blkName='Controllers and Output/Steering Controller/LF steering controller/Convert to output shaft angle';
bio(41).sigName='';
bio(41).portIdx=0;
bio(41).dim=[1,1];
bio(41).sigWidth=1;
bio(41).sigAddress='&p1_B.Converttooutputshaftangle';
bio(41).ndims=2;
bio(41).size=[];


bio(42).blkName='Controllers and Output/Steering Controller/LF steering controller/Convert to output shaft angle (too)';
bio(42).sigName='';
bio(42).portIdx=0;
bio(42).dim=[1,1];
bio(42).sigWidth=1;
bio(42).sigAddress='&p1_B.Converttooutputshaftangletoo';
bio(42).ndims=2;
bio(42).size=[];


bio(43).blkName='Controllers and Output/Steering Controller/LF steering controller/Kd';
bio(43).sigName='';
bio(43).portIdx=0;
bio(43).dim=[1,1];
bio(43).sigWidth=1;
bio(43).sigAddress='&p1_B.Kd';
bio(43).ndims=2;
bio(43).size=[];


bio(44).blkName='Controllers and Output/Steering Controller/LF steering controller/Ki';
bio(44).sigName='';
bio(44).portIdx=0;
bio(44).dim=[1,1];
bio(44).sigWidth=1;
bio(44).sigAddress='&p1_B.Ki';
bio(44).ndims=2;
bio(44).size=[];


bio(45).blkName='Controllers and Output/Steering Controller/LF steering controller/Sum1';
bio(45).sigName='';
bio(45).portIdx=0;
bio(45).dim=[1,1];
bio(45).sigWidth=1;
bio(45).sigAddress='&p1_B.Sum1_l';
bio(45).ndims=2;
bio(45).size=[];


bio(46).blkName='Controllers and Output/Steering Controller/LF steering controller/Sum2';
bio(46).sigName='';
bio(46).portIdx=0;
bio(46).dim=[1,1];
bio(46).sigWidth=1;
bio(46).sigAddress='&p1_B.Sum2_l';
bio(46).ndims=2;
bio(46).size=[];


bio(47).blkName='Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter 2';
bio(47).sigName='';
bio(47).portIdx=0;
bio(47).dim=[1,1];
bio(47).sigWidth=1;
bio(47).sigAddress='&p1_B.DerivativeLowpassfilter2';
bio(47).ndims=2;
bio(47).size=[];


bio(48).blkName='Controllers and Output/Steering Controller/RF steering controller/Convert to output shaft angle';
bio(48).sigName='';
bio(48).portIdx=0;
bio(48).dim=[1,1];
bio(48).sigWidth=1;
bio(48).sigAddress='&p1_B.Converttooutputshaftangle_n';
bio(48).ndims=2;
bio(48).size=[];


bio(49).blkName='Controllers and Output/Steering Controller/RF steering controller/Convert to output shaft angle (too)';
bio(49).sigName='';
bio(49).portIdx=0;
bio(49).dim=[1,1];
bio(49).sigWidth=1;
bio(49).sigAddress='&p1_B.Converttooutputshaftangletoo_e';
bio(49).ndims=2;
bio(49).size=[];


bio(50).blkName='Controllers and Output/Steering Controller/RF steering controller/Kd';
bio(50).sigName='';
bio(50).portIdx=0;
bio(50).dim=[1,1];
bio(50).sigWidth=1;
bio(50).sigAddress='&p1_B.Kd_j';
bio(50).ndims=2;
bio(50).size=[];


bio(51).blkName='Controllers and Output/Steering Controller/RF steering controller/Ki';
bio(51).sigName='';
bio(51).portIdx=0;
bio(51).dim=[1,1];
bio(51).sigWidth=1;
bio(51).sigAddress='&p1_B.Ki_o';
bio(51).ndims=2;
bio(51).size=[];


bio(52).blkName='Controllers and Output/Steering Controller/RF steering controller/Sum1';
bio(52).sigName='';
bio(52).portIdx=0;
bio(52).dim=[1,1];
bio(52).sigWidth=1;
bio(52).sigAddress='&p1_B.Sum1_f';
bio(52).ndims=2;
bio(52).size=[];


bio(53).blkName='Controllers and Output/Steering Controller/RF steering controller/Sum2';
bio(53).sigName='';
bio(53).portIdx=0;
bio(53).dim=[1,1];
bio(53).sigWidth=1;
bio(53).sigAddress='&p1_B.Sum2_o';
bio(53).ndims=2;
bio(53).size=[];


bio(54).blkName='Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter 2';
bio(54).sigName='';
bio(54).portIdx=0;
bio(54).dim=[1,1];
bio(54).sigWidth=1;
bio(54).sigAddress='&p1_B.DerivativeLowpassfilter2_d';
bio(54).ndims=2;
bio(54).size=[];


bio(55).blkName='Controllers and Output/Steering Controller/Steering Geometry & Gearbox Correction/Left Steering Ratio';
bio(55).sigName='';
bio(55).portIdx=0;
bio(55).dim=[1,1];
bio(55).sigWidth=1;
bio(55).sigAddress='&p1_B.LeftSteeringRatio';
bio(55).ndims=2;
bio(55).size=[];


bio(56).blkName='Controllers and Output/Steering Controller/Steering Geometry & Gearbox Correction/Right Steering Ratio';
bio(56).sigName='';
bio(56).portIdx=0;
bio(56).dim=[1,1];
bio(56).sigWidth=1;
bio(56).sigAddress='&p1_B.RightSteeringRatio';
bio(56).ndims=2;
bio(56).size=[];


bio(57).blkName='Controllers and Output/Steering Controller/standby/Enable Controller';
bio(57).sigName='';
bio(57).portIdx=0;
bio(57).dim=[1,1];
bio(57).sigWidth=1;
bio(57).sigAddress='&p1_B.EnableController';
bio(57).ndims=2;
bio(57).size=[];


bio(58).blkName='Controllers and Output/Steering Controller/standby/Derivative & Low-pass filter';
bio(58).sigName='';
bio(58).portIdx=0;
bio(58).dim=[1,1];
bio(58).sigWidth=1;
bio(58).sigAddress='&p1_B.DerivativeLowpassfilter';
bio(58).ndims=2;
bio(58).size=[];


bio(59).blkName='Controllers and Output/Steering Controller/standby/filter';
bio(59).sigName='';
bio(59).portIdx=0;
bio(59).dim=[1,1];
bio(59).sigWidth=1;
bio(59).sigAddress='&p1_B.filter';
bio(59).ndims=2;
bio(59).size=[];


bio(60).blkName='Controllers and Output/Steering Controller/standby/filter1';
bio(60).sigName='';
bio(60).portIdx=0;
bio(60).dim=[1,1];
bio(60).sigWidth=1;
bio(60).sigAddress='&p1_B.filter1';
bio(60).ndims=2;
bio(60).size=[];


bio(61).blkName='Hardware I//O/Input/Encoder Board/encoder board /p1';
bio(61).sigName='';
bio(61).portIdx=0;
bio(61).dim=[1,1];
bio(61).sigWidth=1;
bio(61).sigAddress='&p1_B.encoderboard_o1';
bio(61).ndims=2;
bio(61).size=[];


bio(62).blkName='Hardware I//O/Input/Encoder Board/encoder board /p2';
bio(62).sigName='';
bio(62).portIdx=1;
bio(62).dim=[1,1];
bio(62).sigWidth=1;
bio(62).sigAddress='&p1_B.encoderboard_o2';
bio(62).ndims=2;
bio(62).size=[];


bio(63).blkName='Hardware I//O/Input/Encoder Board/encoder board /p3';
bio(63).sigName='';
bio(63).portIdx=2;
bio(63).dim=[1,1];
bio(63).sigWidth=1;
bio(63).sigAddress='&p1_B.encoderboard_o3';
bio(63).ndims=2;
bio(63).size=[];


bio(64).blkName='Hardware I//O/Input/Encoder Board/encoder board /p4';
bio(64).sigName='';
bio(64).portIdx=3;
bio(64).dim=[1,1];
bio(64).sigWidth=1;
bio(64).sigAddress='&p1_B.encoderboard_o4';
bio(64).ndims=2;
bio(64).size=[];


bio(65).blkName='Hardware I//O/Input/Encoder Board/encoder board /p5';
bio(65).sigName='';
bio(65).portIdx=4;
bio(65).dim=[1,1];
bio(65).sigWidth=1;
bio(65).sigAddress='&p1_B.encoderboard_o5';
bio(65).ndims=2;
bio(65).size=[];


bio(66).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Gear Ratio';
bio(66).sigName='';
bio(66).portIdx=0;
bio(66).dim=[1,1];
bio(66).sigWidth=1;
bio(66).sigAddress='&p1_B.HandwheelGearRatio';
bio(66).ndims=2;
bio(66).size=[];


bio(67).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Harmonic Drive Gear Ratio';
bio(67).sigName='';
bio(67).portIdx=0;
bio(67).dim=[1,1];
bio(67).sigWidth=1;
bio(67).sigAddress='&p1_B.HarmonicDriveGearRatio';
bio(67).ndims=2;
bio(67).size=[];


bio(68).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Harmonic Drive Gear Ratio1';
bio(68).sigName='';
bio(68).portIdx=0;
bio(68).dim=[1,1];
bio(68).sigWidth=1;
bio(68).sigAddress='&p1_B.HarmonicDriveGearRatio1';
bio(68).ndims=2;
bio(68).size=[];


bio(69).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/encoder scaling';
bio(69).sigName='Primary';
bio(69).portIdx=0;
bio(69).dim=[1,1];
bio(69).sigWidth=1;
bio(69).sigAddress='&p1_B.Primary';
bio(69).ndims=2;
bio(69).size=[];


bio(70).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Sum';
bio(70).sigName='';
bio(70).portIdx=0;
bio(70).dim=[1,1];
bio(70).sigWidth=1;
bio(70).sigAddress='&p1_B.Sum';
bio(70).ndims=2;
bio(70).size=[];


bio(71).blkName='Hardware I//O/Input/INS scaling/deg to rad';
bio(71).sigName='yaw';
bio(71).portIdx=0;
bio(71).dim=[1,1];
bio(71).sigWidth=1;
bio(71).sigAddress='&p1_B.yaw';
bio(71).ndims=2;
bio(71).size=[];


bio(72).blkName='Hardware I//O/Input/Rear Wheel Speed Processing/de-quadrature';
bio(72).sigName='';
bio(72).portIdx=0;
bio(72).dim=[2,1];
bio(72).sigWidth=2;
bio(72).sigAddress='&p1_B.dequadrature[0]';
bio(72).ndims=2;
bio(72).size=[];


bio(73).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/gearbox ratio';
bio(73).sigName='';
bio(73).portIdx=0;
bio(73).dim=[2,1];
bio(73).sigWidth=2;
bio(73).sigAddress='&p1_B.gearboxratio[0]';
bio(73).ndims=2;
bio(73).size=[];


bio(74).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Sum1';
bio(74).sigName='';
bio(74).portIdx=0;
bio(74).dim=[2,1];
bio(74).sigWidth=2;
bio(74).sigAddress='&p1_B.Sum1[0]';
bio(74).ndims=2;
bio(74).size=[];


bio(75).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Sum2';
bio(75).sigName='';
bio(75).portIdx=0;
bio(75).dim=[2,1];
bio(75).sigWidth=2;
bio(75).sigAddress='&p1_B.Sum2[0]';
bio(75).ndims=2;
bio(75).size=[];


bio(76).blkName='Hardware I//O/Input/Subsystem1/Sum';
bio(76).sigName='';
bio(76).portIdx=0;
bio(76).dim=[1,1];
bio(76).sigWidth=1;
bio(76).sigAddress='&p1_B.Sum_g';
bio(76).ndims=2;
bio(76).size=[];


bio(77).blkName='Hardware I//O/Input/Subsystem2/Sum';
bio(77).sigName='';
bio(77).portIdx=0;
bio(77).dim=[1,1];
bio(77).sigWidth=1;
bio(77).sigAddress='&p1_B.Sum_l';
bio(77).ndims=2;
bio(77).size=[];


bio(78).blkName='Hardware I//O/Input/VS330 GPS Data/Roll Axis Reversed';
bio(78).sigName='Roll Angle';
bio(78).portIdx=0;
bio(78).dim=[1,1];
bio(78).sigWidth=1;
bio(78).sigAddress='&p1_B.RollAngle';
bio(78).ndims=2;
bio(78).size=[];


bio(79).blkName='Hardware I//O/Input/VS330 GPS Data/Hemisphere VS330/p1';
bio(79).sigName='';
bio(79).portIdx=0;
bio(79).dim=[24,1];
bio(79).sigWidth=24;
bio(79).sigAddress='&p1_B.HemisphereVS330_o1[0]';
bio(79).ndims=2;
bio(79).size=[];


bio(80).blkName='Hardware I//O/Input/VS330 GPS Data/Hemisphere VS330/p2';
bio(80).sigName='';
bio(80).portIdx=1;
bio(80).dim=[1,1];
bio(80).sigWidth=1;
bio(80).sigAddress='&p1_B.HemisphereVS330_o2';
bio(80).ndims=2;
bio(80).size=[];


bio(81).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack/p1';
bio(81).sigName='LF_Fx';
bio(81).portIdx=0;
bio(81).dim=[1,1];
bio(81).sigWidth=1;
bio(81).sigAddress='&p1_B.LF_Fx';
bio(81).ndims=2;
bio(81).size=[];


bio(82).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack/p2';
bio(82).sigName='LF_Fy';
bio(82).portIdx=1;
bio(82).dim=[1,1];
bio(82).sigWidth=1;
bio(82).sigAddress='&p1_B.LF_Fy';
bio(82).ndims=2;
bio(82).size=[];


bio(83).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack/p3';
bio(83).sigName='LF_Fz';
bio(83).portIdx=2;
bio(83).dim=[1,1];
bio(83).sigWidth=1;
bio(83).sigAddress='&p1_B.LF_Fz';
bio(83).ndims=2;
bio(83).size=[];


bio(84).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack/p4';
bio(84).sigName='LF_Mx';
bio(84).portIdx=3;
bio(84).dim=[1,1];
bio(84).sigWidth=1;
bio(84).sigAddress='&p1_B.LF_Mx';
bio(84).ndims=2;
bio(84).size=[];


bio(85).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack/p5';
bio(85).sigName='';
bio(85).portIdx=4;
bio(85).dim=[1,1];
bio(85).sigWidth=1;
bio(85).sigAddress='&p1_B.CANUnpack_o5';
bio(85).ndims=2;
bio(85).size=[];


bio(86).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack1/p1';
bio(86).sigName='LF_My';
bio(86).portIdx=0;
bio(86).dim=[1,1];
bio(86).sigWidth=1;
bio(86).sigAddress='&p1_B.LF_My';
bio(86).ndims=2;
bio(86).size=[];


bio(87).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack1/p2';
bio(87).sigName='LF_Mz';
bio(87).portIdx=1;
bio(87).dim=[1,1];
bio(87).sigWidth=1;
bio(87).sigAddress='&p1_B.LF_Mz';
bio(87).ndims=2;
bio(87).size=[];


bio(88).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack1/p3';
bio(88).sigName='LF_omega';
bio(88).portIdx=2;
bio(88).dim=[1,1];
bio(88).sigWidth=1;
bio(88).sigAddress='&p1_B.LF_omega';
bio(88).ndims=2;
bio(88).size=[];


bio(89).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack1/p4';
bio(89).sigName='LF_theta';
bio(89).portIdx=3;
bio(89).dim=[1,1];
bio(89).sigWidth=1;
bio(89).sigAddress='&p1_B.LF_theta';
bio(89).ndims=2;
bio(89).size=[];


bio(90).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack1/p5';
bio(90).sigName='';
bio(90).portIdx=4;
bio(90).dim=[1,1];
bio(90).sigWidth=1;
bio(90).sigAddress='&p1_B.CANUnpack1_o5';
bio(90).ndims=2;
bio(90).size=[];


bio(91).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack2/p1';
bio(91).sigName='LF_ax';
bio(91).portIdx=0;
bio(91).dim=[1,1];
bio(91).sigWidth=1;
bio(91).sigAddress='&p1_B.LF_ax';
bio(91).ndims=2;
bio(91).size=[];


bio(92).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack2/p2';
bio(92).sigName='LF_ay';
bio(92).portIdx=1;
bio(92).dim=[1,1];
bio(92).sigWidth=1;
bio(92).sigAddress='&p1_B.LF_ay';
bio(92).ndims=2;
bio(92).size=[];


bio(93).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack2/p3';
bio(93).sigName='';
bio(93).portIdx=2;
bio(93).dim=[1,1];
bio(93).sigWidth=1;
bio(93).sigAddress='&p1_B.CANUnpack2_o3';
bio(93).ndims=2;
bio(93).size=[];


bio(94).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack3/p1';
bio(94).sigName='RF_Fx';
bio(94).portIdx=0;
bio(94).dim=[1,1];
bio(94).sigWidth=1;
bio(94).sigAddress='&p1_B.RF_Fx';
bio(94).ndims=2;
bio(94).size=[];


bio(95).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack3/p2';
bio(95).sigName='RF_Fy';
bio(95).portIdx=1;
bio(95).dim=[1,1];
bio(95).sigWidth=1;
bio(95).sigAddress='&p1_B.RF_Fy';
bio(95).ndims=2;
bio(95).size=[];


bio(96).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack3/p3';
bio(96).sigName='RF_Fz';
bio(96).portIdx=2;
bio(96).dim=[1,1];
bio(96).sigWidth=1;
bio(96).sigAddress='&p1_B.RF_Fz';
bio(96).ndims=2;
bio(96).size=[];


bio(97).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack3/p4';
bio(97).sigName='RF_Mx';
bio(97).portIdx=3;
bio(97).dim=[1,1];
bio(97).sigWidth=1;
bio(97).sigAddress='&p1_B.RF_Mx';
bio(97).ndims=2;
bio(97).size=[];


bio(98).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack3/p5';
bio(98).sigName='';
bio(98).portIdx=4;
bio(98).dim=[1,1];
bio(98).sigWidth=1;
bio(98).sigAddress='&p1_B.CANUnpack3_o5';
bio(98).ndims=2;
bio(98).size=[];


bio(99).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack4/p1';
bio(99).sigName='RF_My';
bio(99).portIdx=0;
bio(99).dim=[1,1];
bio(99).sigWidth=1;
bio(99).sigAddress='&p1_B.RF_My';
bio(99).ndims=2;
bio(99).size=[];


bio(100).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack4/p2';
bio(100).sigName='RF_Mz';
bio(100).portIdx=1;
bio(100).dim=[1,1];
bio(100).sigWidth=1;
bio(100).sigAddress='&p1_B.RF_Mz';
bio(100).ndims=2;
bio(100).size=[];


bio(101).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack4/p3';
bio(101).sigName='RF_omega';
bio(101).portIdx=2;
bio(101).dim=[1,1];
bio(101).sigWidth=1;
bio(101).sigAddress='&p1_B.RF_omega';
bio(101).ndims=2;
bio(101).size=[];


bio(102).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack4/p4';
bio(102).sigName='RF_theta';
bio(102).portIdx=3;
bio(102).dim=[1,1];
bio(102).sigWidth=1;
bio(102).sigAddress='&p1_B.RF_theta';
bio(102).ndims=2;
bio(102).size=[];


bio(103).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack4/p5';
bio(103).sigName='';
bio(103).portIdx=4;
bio(103).dim=[1,1];
bio(103).sigWidth=1;
bio(103).sigAddress='&p1_B.CANUnpack4_o5';
bio(103).ndims=2;
bio(103).size=[];


bio(104).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack5/p1';
bio(104).sigName='RF_ax';
bio(104).portIdx=0;
bio(104).dim=[1,1];
bio(104).sigWidth=1;
bio(104).sigAddress='&p1_B.RF_ax';
bio(104).ndims=2;
bio(104).size=[];


bio(105).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack5/p2';
bio(105).sigName='RF_ay';
bio(105).portIdx=1;
bio(105).dim=[1,1];
bio(105).sigWidth=1;
bio(105).sigAddress='&p1_B.RF_ay';
bio(105).ndims=2;
bio(105).size=[];


bio(106).blkName='Hardware I//O/Input/Wheel Force Transducers/CAN Unpack5/p3';
bio(106).sigName='';
bio(106).portIdx=2;
bio(106).dim=[1,1];
bio(106).sigWidth=1;
bio(106).sigAddress='&p1_B.CANUnpack5_o3';
bio(106).ndims=2;
bio(106).size=[];


bio(107).blkName='Hardware I//O/Input/Wheel Force Transducers/Receive /p1';
bio(107).sigName='';
bio(107).portIdx=0;
bio(107).dim=[1,1];
bio(107).sigWidth=1;
bio(107).sigAddress='&p1_B.Receive_o1';
bio(107).ndims=2;
bio(107).size=[];


bio(108).blkName='Hardware I//O/Input/Wheel Force Transducers/Receive /p2';
bio(108).sigName='';
bio(108).portIdx=1;
bio(108).dim=[1,1];
bio(108).sigWidth=1;
bio(108).sigAddress='&p1_B.Receive_o2';
bio(108).ndims=2;
bio(108).size=[];


bio(109).blkName='Hardware I//O/Input/Wheel Force Transducers/Receive /p3';
bio(109).sigName='';
bio(109).portIdx=2;
bio(109).dim=[1,1];
bio(109).sigWidth=1;
bio(109).sigAddress='&p1_B.Receive_o3';
bio(109).ndims=2;
bio(109).size=[];


bio(110).blkName='Hardware I//O/Input/Wheel Force Transducers/Receive /p4';
bio(110).sigName='';
bio(110).portIdx=3;
bio(110).dim=[1,1];
bio(110).sigWidth=1;
bio(110).sigAddress='&p1_B.Receive_o4';
bio(110).ndims=2;
bio(110).size=[];


bio(111).blkName='Hardware I//O/Input/Wheel Force Transducers/Receive /p5';
bio(111).sigName='';
bio(111).portIdx=4;
bio(111).dim=[1,1];
bio(111).sigWidth=1;
bio(111).sigAddress='&p1_B.Receive_o5';
bio(111).ndims=2;
bio(111).size=[];


bio(112).blkName='Hardware I//O/Input/Wheel Force Transducers/Receive /p6';
bio(112).sigName='';
bio(112).portIdx=5;
bio(112).dim=[1,1];
bio(112).sigWidth=1;
bio(112).sigAddress='&p1_B.Receive_o6';
bio(112).ndims=2;
bio(112).size=[];


bio(113).blkName='Hardware I//O/Input/current scaling/Sum';
bio(113).sigName='';
bio(113).portIdx=0;
bio(113).dim=[2,1];
bio(113).sigWidth=2;
bio(113).sigAddress='&p1_B.Sum_h[0]';
bio(113).ndims=2;
bio(113).size=[];


bio(114).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable';
bio(114).sigName='';
bio(114).portIdx=0;
bio(114).dim=[1,1];
bio(114).sigWidth=1;
bio(114).sigAddress='&p1_B.Disable';
bio(114).ndims=2;
bio(114).size=[];


bio(115).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable1';
bio(115).sigName='';
bio(115).portIdx=0;
bio(115).dim=[1,1];
bio(115).sigWidth=1;
bio(115).sigAddress='&p1_B.Disable1';
bio(115).ndims=2;
bio(115).size=[];


bio(116).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable3';
bio(116).sigName='';
bio(116).portIdx=0;
bio(116).dim=[1,1];
bio(116).sigWidth=1;
bio(116).sigAddress='&p1_B.Disable3';
bio(116).ndims=2;
bio(116).size=[];


bio(117).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable4';
bio(117).sigName='';
bio(117).portIdx=0;
bio(117).dim=[1,1];
bio(117).sigWidth=1;
bio(117).sigAddress='&p1_B.Disable4';
bio(117).ndims=2;
bio(117).size=[];


bio(118).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable6';
bio(118).sigName='Left';
bio(118).portIdx=0;
bio(118).dim=[1,1];
bio(118).sigWidth=1;
bio(118).sigAddress='&p1_B.Left_p';
bio(118).ndims=2;
bio(118).size=[];


bio(119).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable7';
bio(119).sigName='Right';
bio(119).portIdx=0;
bio(119).dim=[1,1];
bio(119).sigWidth=1;
bio(119).sigAddress='&p1_B.Right_b';
bio(119).ndims=2;
bio(119).size=[];


bio(120).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Damping';
bio(120).sigName='Left';
bio(120).portIdx=0;
bio(120).dim=[1,1];
bio(120).sigWidth=1;
bio(120).sigAddress='&p1_B.Left_l';
bio(120).ndims=2;
bio(120).size=[];


bio(121).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Damping1';
bio(121).sigName='Right';
bio(121).portIdx=0;
bio(121).dim=[1,1];
bio(121).sigWidth=1;
bio(121).sigAddress='&p1_B.Right_e';
bio(121).ndims=2;
bio(121).size=[];


bio(122).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Friction';
bio(122).sigName='';
bio(122).portIdx=0;
bio(122).dim=[1,1];
bio(122).sigWidth=1;
bio(122).sigAddress='&p1_B.WheelFriction';
bio(122).ndims=2;
bio(122).size=[];


bio(123).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Friction1';
bio(123).sigName='';
bio(123).portIdx=0;
bio(123).dim=[1,1];
bio(123).sigWidth=1;
bio(123).sigAddress='&p1_B.WheelFriction1';
bio(123).ndims=2;
bio(123).size=[];


bio(124).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Inertia';
bio(124).sigName='Left';
bio(124).portIdx=0;
bio(124).dim=[1,1];
bio(124).sigWidth=1;
bio(124).sigAddress='&p1_B.Left_e';
bio(124).ndims=2;
bio(124).size=[];


bio(125).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Inertia1';
bio(125).sigName='Right';
bio(125).portIdx=0;
bio(125).dim=[1,1];
bio(125).sigWidth=1;
bio(125).sigAddress='&p1_B.Right_h';
bio(125).ndims=2;
bio(125).size=[];


bio(126).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Product';
bio(126).sigName='Left';
bio(126).portIdx=0;
bio(126).dim=[1,1];
bio(126).sigWidth=1;
bio(126).sigAddress='&p1_B.Left_k';
bio(126).ndims=2;
bio(126).size=[];


bio(127).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Product1';
bio(127).sigName='Right';
bio(127).portIdx=0;
bio(127).dim=[1,1];
bio(127).sigWidth=1;
bio(127).sigAddress='&p1_B.Right_k';
bio(127).ndims=2;
bio(127).size=[];


bio(128).blkName='Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Left Arm Length Look-up';
bio(128).sigName='';
bio(128).portIdx=0;
bio(128).dim=[1,1];
bio(128).sigWidth=1;
bio(128).sigAddress='&p1_B.LeftArmLengthLookup';
bio(128).ndims=2;
bio(128).size=[];


bio(129).blkName='Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Right Arm Length Look-up';
bio(129).sigName='';
bio(129).portIdx=0;
bio(129).dim=[1,1];
bio(129).sigWidth=1;
bio(129).sigAddress='&p1_B.RightArmLengthLookup';
bio(129).ndims=2;
bio(129).size=[];


bio(130).blkName='Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Product';
bio(130).sigName='Left';
bio(130).portIdx=0;
bio(130).dim=[1,1];
bio(130).sigWidth=1;
bio(130).sigAddress='&p1_B.Left_o';
bio(130).ndims=2;
bio(130).size=[];


bio(131).blkName='Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Product1';
bio(131).sigName='Right';
bio(131).portIdx=0;
bio(131).dim=[1,1];
bio(131).sigWidth=1;
bio(131).sigAddress='&p1_B.Right_p';
bio(131).ndims=2;
bio(131).size=[];


bio(132).blkName='Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Left Steer Angle Look-up';
bio(132).sigName='Left';
bio(132).portIdx=0;
bio(132).dim=[1,1];
bio(132).sigWidth=1;
bio(132).sigAddress='&p1_B.Left';
bio(132).ndims=2;
bio(132).size=[];


bio(133).blkName='Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Right Steer Angle Look-up';
bio(133).sigName='Right';
bio(133).portIdx=0;
bio(133).dim=[1,1];
bio(133).sigWidth=1;
bio(133).sigAddress='&p1_B.Right';
bio(133).ndims=2;
bio(133).size=[];


bio(134).blkName='Sensor Data and Estimation/Vehicle State Estimator/Select: Yaw Rate/Gain';
bio(134).sigName='';
bio(134).portIdx=0;
bio(134).dim=[1,1];
bio(134).sigWidth=1;
bio(134).sigAddress='&p1_B.Gain';
bio(134).ndims=2;
bio(134).size=[];


bio(135).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad1';
bio(135).sigName='Yaw Rate';
bio(135).portIdx=0;
bio(135).dim=[1,1];
bio(135).sigWidth=1;
bio(135).sigAddress='&p1_ConstB.YawRate';
bio(135).ndims=2;
bio(135).size=[];


bio(136).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad2';
bio(136).sigName='Yaw Angle';
bio(136).portIdx=0;
bio(136).dim=[1,1];
bio(136).sigWidth=1;
bio(136).sigAddress='&p1_ConstB.YawAngle';
bio(136).ndims=2;
bio(136).size=[];


bio(137).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad3';
bio(137).sigName='Roll Angle';
bio(137).portIdx=0;
bio(137).dim=[1,1];
bio(137).sigWidth=1;
bio(137).sigAddress='&p1_ConstB.RollAngle';
bio(137).ndims=2;
bio(137).size=[];


bio(138).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad4';
bio(138).sigName='Roll Rate';
bio(138).portIdx=0;
bio(138).dim=[1,1];
bio(138).sigWidth=1;
bio(138).sigAddress='&p1_ConstB.RollRate';
bio(138).ndims=2;
bio(138).size=[];


bio(139).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad5';
bio(139).sigName='Yaw Gyro Bias';
bio(139).portIdx=0;
bio(139).dim=[1,1];
bio(139).sigWidth=1;
bio(139).sigAddress='&p1_ConstB.YawGyroBias';
bio(139).ndims=2;
bio(139).size=[];


bio(140).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad6';
bio(140).sigName='Roll Gyro Bias';
bio(140).portIdx=0;
bio(140).dim=[1,1];
bio(140).sigWidth=1;
bio(140).sigAddress='&p1_ConstB.RollGyroBias';
bio(140).ndims=2;
bio(140).size=[];


bio(141).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad7';
bio(141).sigName='Sideslip';
bio(141).portIdx=0;
bio(141).dim=[1,1];
bio(141).sigWidth=1;
bio(141).sigAddress='&p1_ConstB.Sideslip';
bio(141).ndims=2;
bio(141).size=[];


bio(142).blkName='User Controllers/Cruise Control/Pedal Latch/In1';
bio(142).sigName='';
bio(142).portIdx=0;
bio(142).dim=[1,1];
bio(142).sigWidth=1;
bio(142).sigAddress='&p1_B.PedalLatch.In1';
bio(142).ndims=2;
bio(142).size=[];


bio(143).blkName='User Controllers/Cruise Control/Speed Latch/In1';
bio(143).sigName='';
bio(143).portIdx=0;
bio(143).dim=[1,1];
bio(143).sigWidth=1;
bio(143).sigAddress='&p1_B.SpeedLatch.In1';
bio(143).ndims=2;
bio(143).size=[];


bio(144).blkName='User Controllers/Steering Controller/HAL-9000/Clock';
bio(144).sigName='';
bio(144).portIdx=0;
bio(144).dim=[1,1];
bio(144).sigWidth=1;
bio(144).sigAddress='&p1_B.Clock';
bio(144).ndims=2;
bio(144).size=[];


bio(145).blkName='User Controllers/Steering Controller/HAL-9000/Product';
bio(145).sigName='';
bio(145).portIdx=0;
bio(145).dim=[1,1];
bio(145).sigWidth=1;
bio(145).sigAddress='&p1_B.Product_k';
bio(145).ndims=2;
bio(145).size=[];


bio(146).blkName='User Controllers/Steering Controller/HAL-9000/Product1';
bio(146).sigName='';
bio(146).portIdx=0;
bio(146).dim=[1,1];
bio(146).sigWidth=1;
bio(146).sigAddress='&p1_B.Product1';
bio(146).ndims=2;
bio(146).size=[];


bio(147).blkName='User Controllers/Steering Controller/HAL-9000/Sum';
bio(147).sigName='';
bio(147).portIdx=0;
bio(147).dim=[1,1];
bio(147).sigWidth=1;
bio(147).sigAddress='&p1_B.Sum_hm';
bio(147).ndims=2;
bio(147).size=[];


bio(148).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Encoder Initialization/Encoder Initializer';
bio(148).sigName='';
bio(148).portIdx=0;
bio(148).dim=[1,1];
bio(148).sigWidth=1;
bio(148).sigAddress='&p1_B.EncoderInitializer';
bio(148).ndims=2;
bio(148).size=[];


bio(149).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Encoder Initialization/Unwrapper';
bio(149).sigName='';
bio(149).portIdx=0;
bio(149).dim=[1,1];
bio(149).sigWidth=1;
bio(149).sigAddress='&p1_B.Unwrapper';
bio(149).ndims=2;
bio(149).size=[];


bio(150).blkName='Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc/Unwrapping';
bio(150).sigName='';
bio(150).portIdx=0;
bio(150).dim=[1,1];
bio(150).sigWidth=1;
bio(150).sigAddress='&p1_B.Unwrapping';
bio(150).ndims=2;
bio(150).size=[];


bio(151).blkName='Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc1/Unwrapping';
bio(151).sigName='';
bio(151).portIdx=0;
bio(151).dim=[1,1];
bio(151).sigWidth=1;
bio(151).sigAddress='&p1_B.Unwrapping_n';
bio(151).ndims=2;
bio(151).size=[];


bio(152).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Left/Encoder Initializer';
bio(152).sigName='';
bio(152).portIdx=0;
bio(152).dim=[1,1];
bio(152).sigWidth=1;
bio(152).sigAddress='&p1_B.EncoderInitializer_i';
bio(152).ndims=2;
bio(152).size=[];


bio(153).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Left/Unwrapper';
bio(153).sigName='';
bio(153).portIdx=0;
bio(153).dim=[1,1];
bio(153).sigWidth=1;
bio(153).sigAddress='&p1_B.Unwrapper_o';
bio(153).ndims=2;
bio(153).size=[];


bio(154).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Right/Encoder Initializer';
bio(154).sigName='';
bio(154).portIdx=0;
bio(154).dim=[1,1];
bio(154).sigWidth=1;
bio(154).sigAddress='&p1_B.EncoderInitializer_k';
bio(154).ndims=2;
bio(154).size=[];


bio(155).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Right/Unwrapper';
bio(155).sigName='';
bio(155).portIdx=0;
bio(155).dim=[1,1];
bio(155).sigWidth=1;
bio(155).sigAddress='&p1_B.Unwrapper_a';
bio(155).ndims=2;
bio(155).size=[];


bio(156).blkName='Hardware I//O/Input/VS330 GPS Data/Subsystem/Product';
bio(156).sigName='';
bio(156).portIdx=0;
bio(156).dim=[1,1];
bio(156).sigWidth=1;
bio(156).sigAddress='&p1_B.Product';
bio(156).ndims=2;
bio(156).size=[];


bio(157).blkName='Hardware I//O/Input/VS330 GPS Data/Subsystem/Product4';
bio(157).sigName='';
bio(157).portIdx=0;
bio(157).dim=[1,1];
bio(157).sigWidth=1;
bio(157).sigAddress='&p1_B.Product4';
bio(157).ndims=2;
bio(157).size=[];


bio(158).blkName='Hardware I//O/Input/VS330 GPS Data/Subsystem/Product5';
bio(158).sigName='';
bio(158).portIdx=0;
bio(158).dim=[1,1];
bio(158).sigWidth=1;
bio(158).sigAddress='&p1_B.Product5';
bio(158).ndims=2;
bio(158).size=[];


bio(159).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Left Steer Angle Look-up1';
bio(159).sigName='Left';
bio(159).portIdx=0;
bio(159).dim=[1,1];
bio(159).sigWidth=1;
bio(159).sigAddress='&p1_B.Left_j';
bio(159).ndims=2;
bio(159).size=[];


bio(160).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Right Steer Angle Look-up1';
bio(160).sigName='Right';
bio(160).portIdx=0;
bio(160).dim=[1,1];
bio(160).sigWidth=1;
bio(160).sigAddress='&p1_B.Right_h0';
bio(160).ndims=2;
bio(160).size=[];


bio(161).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations/Diff';
bio(161).sigName='';
bio(161).portIdx=0;
bio(161).dim=[1,1];
bio(161).sigWidth=1;
bio(161).sigAddress='&p1_B.Diff';
bio(161).ndims=2;
bio(161).size=[];


bio(162).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations/TSamp';
bio(162).sigName='';
bio(162).portIdx=0;
bio(162).dim=[1,1];
bio(162).sigWidth=1;
bio(162).sigAddress='&p1_B.TSamp';
bio(162).ndims=2;
bio(162).size=[];


bio(163).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations1/Diff';
bio(163).sigName='';
bio(163).portIdx=0;
bio(163).dim=[1,1];
bio(163).sigWidth=1;
bio(163).sigAddress='&p1_B.Diff_b';
bio(163).ndims=2;
bio(163).size=[];


bio(164).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations1/TSamp';
bio(164).sigName='';
bio(164).portIdx=0;
bio(164).dim=[1,1];
bio(164).sigWidth=1;
bio(164).sigAddress='&p1_B.TSamp_i';
bio(164).ndims=2;
bio(164).size=[];


bio(165).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities/Diff';
bio(165).sigName='';
bio(165).portIdx=0;
bio(165).dim=[1,1];
bio(165).sigWidth=1;
bio(165).sigAddress='&p1_B.Diff_o';
bio(165).ndims=2;
bio(165).size=[];


bio(166).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities/TSamp';
bio(166).sigName='';
bio(166).portIdx=0;
bio(166).dim=[1,1];
bio(166).sigWidth=1;
bio(166).sigAddress='&p1_B.TSamp_a';
bio(166).ndims=2;
bio(166).size=[];


bio(167).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities1/Diff';
bio(167).sigName='';
bio(167).portIdx=0;
bio(167).dim=[1,1];
bio(167).sigWidth=1;
bio(167).sigAddress='&p1_B.Diff_ok';
bio(167).ndims=2;
bio(167).size=[];


bio(168).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities1/TSamp';
bio(168).sigName='';
bio(168).portIdx=0;
bio(168).dim=[1,1];
bio(168).sigWidth=1;
bio(168).sigAddress='&p1_B.TSamp_h';
bio(168).ndims=2;
bio(168).size=[];


bio(169).blkName='User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay';
bio(169).sigName='';
bio(169).portIdx=0;
bio(169).dim=[1,1];
bio(169).sigWidth=1;
bio(169).sigAddress='&p1_B.UnitDelay';
bio(169).ndims=2;
bio(169).size=[];


bio(170).blkName='User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay1';
bio(170).sigName='';
bio(170).portIdx=0;
bio(170).dim=[1,1];
bio(170).sigWidth=1;
bio(170).sigAddress='&p1_B.UnitDelay1';
bio(170).ndims=2;
bio(170).size=[];


bio(171).blkName='User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay2';
bio(171).sigName='';
bio(171).portIdx=0;
bio(171).dim=[1,1];
bio(171).sigWidth=1;
bio(171).sigAddress='&p1_B.UnitDelay2';
bio(171).ndims=2;
bio(171).size=[];


bio(172).blkName='User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay3';
bio(172).sigName='';
bio(172).portIdx=0;
bio(172).dim=[1,1];
bio(172).sigWidth=1;
bio(172).sigAddress='&p1_B.UnitDelay3';
bio(172).ndims=2;
bio(172).size=[];


bio(173).blkName='User Controllers/Cruise Control/Enable Logic/Latch1/In1';
bio(173).sigName='';
bio(173).portIdx=0;
bio(173).dim=[1,1];
bio(173).sigWidth=1;
bio(173).sigAddress='&p1_B.Latch1.In1';
bio(173).ndims=2;
bio(173).size=[];


bio(174).blkName='User Controllers/Steering Controller/HAL-9000/Latch/In1';
bio(174).sigName='';
bio(174).portIdx=0;
bio(174).dim=[1,1];
bio(174).sigWidth=1;
bio(174).sigAddress='&p1_B.In1';
bio(174).ndims=2;
bio(174).size=[];


bio(175).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Logical Operator4';
bio(175).sigName='';
bio(175).portIdx=0;
bio(175).dim=[1,1];
bio(175).sigWidth=1;
bio(175).sigAddress='&p1_B.LogicalOperator4';
bio(175).ndims=2;
bio(175).size=[];


bio(176).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Memory3';
bio(176).sigName='';
bio(176).portIdx=0;
bio(176).dim=[1,1];
bio(176).sigWidth=1;
bio(176).sigAddress='&p1_B.Memory3';
bio(176).ndims=2;
bio(176).size=[];


bio(177).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Enable Timer/Relational Operator';
bio(177).sigName='';
bio(177).portIdx=0;
bio(177).dim=[1,1];
bio(177).sigWidth=1;
bio(177).sigAddress='&p1_B.RelationalOperator';
bio(177).ndims=2;
bio(177).size=[];


bio(178).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Memory2';
bio(178).sigName='';
bio(178).portIdx=0;
bio(178).dim=[1,1];
bio(178).sigWidth=1;
bio(178).sigAddress='&p1_B.Memory2';
bio(178).ndims=2;
bio(178).size=[];


bio(179).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Switch Over/Switch';
bio(179).sigName='';
bio(179).portIdx=0;
bio(179).dim=[1,1];
bio(179).sigWidth=1;
bio(179).sigAddress='&p1_B.Switch';
bio(179).ndims=2;
bio(179).size=[];


bio(180).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/Data Type Conversion';
bio(180).sigName='';
bio(180).portIdx=0;
bio(180).dim=[1,1];
bio(180).sigWidth=1;
bio(180).sigAddress='&p1_B.DataTypeConversion';
bio(180).ndims=2;
bio(180).size=[];


bio(181).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/Delay function';
bio(181).sigName='';
bio(181).portIdx=0;
bio(181).dim=[1,1];
bio(181).sigWidth=1;
bio(181).sigAddress='&p1_B.Delayfunction';
bio(181).ndims=2;
bio(181).size=[];


bio(182).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/S-Function4';
bio(182).sigName='';
bio(182).portIdx=0;
bio(182).dim=[1,1];
bio(182).sigWidth=1;
bio(182).sigAddress='&p1_B.SFunction4';
bio(182).ndims=2;
bio(182).size=[];


bio(183).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/Switch2';
bio(183).sigName='';
bio(183).portIdx=0;
bio(183).dim=[1,1];
bio(183).sigWidth=1;
bio(183).sigAddress='&p1_B.Switch2';
bio(183).ndims=2;
bio(183).size=[];


bio(184).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/Switch3.5';
bio(184).sigName='';
bio(184).portIdx=0;
bio(184).dim=[1,1];
bio(184).sigWidth=1;
bio(184).sigAddress='&p1_B.Switch35';
bio(184).ndims=2;
bio(184).size=[];


bio(185).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter/Relational Operator2';
bio(185).sigName='';
bio(185).portIdx=0;
bio(185).dim=[1,1];
bio(185).sigWidth=1;
bio(185).sigAddress='&p1_B.RelationalOperator2';
bio(185).ndims=2;
bio(185).size=[];


bio(186).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter1/Relational Operator2';
bio(186).sigName='';
bio(186).portIdx=0;
bio(186).dim=[1,1];
bio(186).sigWidth=1;
bio(186).sigAddress='&p1_B.RelationalOperator2_h';
bio(186).ndims=2;
bio(186).size=[];


bio(187).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Enabled Subsystem/In1';
bio(187).sigName='';
bio(187).portIdx=0;
bio(187).dim=[1,1];
bio(187).sigWidth=1;
bio(187).sigAddress='&p1_B.EnabledSubsystem_i.In1';
bio(187).ndims=2;
bio(187).size=[];


bio(188).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Enabled Subsystem1/In1';
bio(188).sigName='';
bio(188).portIdx=0;
bio(188).dim=[1,1];
bio(188).sigWidth=1;
bio(188).sigAddress='&p1_B.EnabledSubsystem1.In1';
bio(188).ndims=2;
bio(188).size=[];


bio(189).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Nearly the Same/Relational Operator';
bio(189).sigName='';
bio(189).portIdx=0;
bio(189).dim=[1,1];
bio(189).sigWidth=1;
bio(189).sigAddress='&p1_B.RelationalOperator_m';
bio(189).ndims=2;
bio(189).size=[];


bio(190).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Nearly the Same2/Relational Operator';
bio(190).sigName='';
bio(190).portIdx=0;
bio(190).dim=[1,1];
bio(190).sigWidth=1;
bio(190).sigAddress='&p1_B.RelationalOperator_k';
bio(190).ndims=2;
bio(190).size=[];


bio(191).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Latch/Enabled Subsystem/In1';
bio(191).sigName='';
bio(191).portIdx=0;
bio(191).dim=[1,1];
bio(191).sigWidth=1;
bio(191).sigAddress='&p1_B.EnabledSubsystem.In1';
bio(191).ndims=2;
bio(191).size=[];


bio(192).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Latch1/Enabled Subsystem/In1';
bio(192).sigName='';
bio(192).portIdx=0;
bio(192).dim=[1,1];
bio(192).sigWidth=1;
bio(192).sigAddress='&p1_B.EnabledSubsystem_h.In1';
bio(192).ndims=2;
bio(192).size=[];


function len = getlenBIO
len = 192;

