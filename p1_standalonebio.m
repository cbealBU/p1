function bio=p1_standalonebio
bio = [];
bio(1).blkName='Data Logging/Named Mux';
bio(1).sigName='';
bio(1).portIdx=0;
bio(1).dim=[119,1];
bio(1).sigWidth=119;
bio(1).sigAddress='&p1_standalone_B.NamedMux[0]';
bio(1).ndims=2;
bio(1).size=[];

bio(getlenBIO) = bio(1);

bio(2).blkName='User Controllers/Gain';
bio(2).sigName='';
bio(2).portIdx=0;
bio(2).dim=[2,1];
bio(2).sigWidth=2;
bio(2).sigAddress='&p1_standalone_B.Gain_m[0]';
bio(2).ndims=2;
bio(2).size=[];


bio(3).blkName='Controllers and Output/Steering Controller/Product';
bio(3).sigName='';
bio(3).portIdx=0;
bio(3).dim=[2,1];
bio(3).sigWidth=2;
bio(3).sigAddress='&p1_standalone_B.Product_b[0]';
bio(3).ndims=2;
bio(3).size=[];


bio(4).blkName='Controllers and Output/UI Controller/Heartbeat Signal';
bio(4).sigName='';
bio(4).portIdx=0;
bio(4).dim=[1,1];
bio(4).sigWidth=1;
bio(4).sigAddress='&p1_standalone_B.HeartbeatSignal';
bio(4).ndims=2;
bio(4).size=[];


bio(5).blkName='Hardware I//O/Input/Load Cell Scaling';
bio(5).sigName='Front Load Cells';
bio(5).portIdx=0;
bio(5).dim=[2,1];
bio(5).sigWidth=2;
bio(5).sigAddress='&p1_standalone_B.FrontLoadCells[0]';
bio(5).ndims=2;
bio(5).size=[];


bio(6).blkName='Hardware I//O/Input/Load Cell Scaling Rear';
bio(6).sigName='Rear Load Cells';
bio(6).portIdx=0;
bio(6).dim=[2,1];
bio(6).sigWidth=2;
bio(6).sigAddress='&p1_standalone_B.RearLoadCells[0]';
bio(6).ndims=2;
bio(6).size=[];


bio(7).blkName='Hardware I//O/Input/encoder scaling';
bio(7).sigName='Angles';
bio(7).portIdx=0;
bio(7).dim=[2,1];
bio(7).sigWidth=2;
bio(7).sigAddress='&p1_standalone_B.Angles[0]';
bio(7).ndims=2;
bio(7).size=[];


bio(8).blkName='Hardware I//O/Input/voltage scaling';
bio(8).sigName='Voltages';
bio(8).portIdx=0;
bio(8).dim=[2,1];
bio(8).sigWidth=2;
bio(8).sigAddress='&p1_standalone_B.Voltages[0]';
bio(8).ndims=2;
bio(8).size=[];


bio(9).blkName='Hardware I//O/Input/Absolute Encoder';
bio(9).sigName='';
bio(9).portIdx=0;
bio(9).dim=[1,1];
bio(9).sigWidth=1;
bio(9).sigAddress='&p1_standalone_B.AbsoluteEncoder';
bio(9).ndims=2;
bio(9).size=[];


bio(10).blkName='Hardware I//O/Input/Analog Input (DAS)';
bio(10).sigName='';
bio(10).portIdx=0;
bio(10).dim=[16,1];
bio(10).sigWidth=16;
bio(10).sigAddress='&p1_standalone_B.AnalogInputDAS[0]';
bio(10).ndims=2;
bio(10).size=[];


bio(11).blkName='Hardware I//O/Input/Analog Input (VSBC)';
bio(11).sigName='';
bio(11).portIdx=0;
bio(11).dim=[8,1];
bio(11).sigWidth=8;
bio(11).sigAddress='&p1_standalone_B.AnalogInputVSBC[0]';
bio(11).ndims=2;
bio(11).size=[];


bio(12).blkName='Hardware I//O/Input/Ruby-MM/p1';
bio(12).sigName='Fault Switch';
bio(12).portIdx=0;
bio(12).dim=[1,1];
bio(12).sigWidth=1;
bio(12).sigAddress='&p1_standalone_B.FaultSwitch';
bio(12).ndims=2;
bio(12).size=[];


bio(13).blkName='Hardware I//O/Input/Ruby-MM/p2';
bio(13).sigName='Auto-Steer Switch';
bio(13).portIdx=1;
bio(13).dim=[1,1];
bio(13).sigWidth=1;
bio(13).sigAddress='&p1_standalone_B.AutoSteerSwitch';
bio(13).ndims=2;
bio(13).size=[];


bio(14).blkName='Hardware I//O/Input/Ruby-MM/p3';
bio(14).sigName='';
bio(14).portIdx=2;
bio(14).dim=[1,1];
bio(14).sigWidth=1;
bio(14).sigAddress='&p1_standalone_B.RubyMM_o3';
bio(14).ndims=2;
bio(14).size=[];


bio(15).blkName='Hardware I//O/Input/Ruby-MM/p4';
bio(15).sigName='';
bio(15).portIdx=3;
bio(15).dim=[1,1];
bio(15).sigWidth=1;
bio(15).sigAddress='&p1_standalone_B.RubyMM_o4';
bio(15).ndims=2;
bio(15).size=[];


bio(16).blkName='Hardware I//O/Input/Wheelspeed Sensors';
bio(16).sigName='Front Wheel Speeds';
bio(16).portIdx=0;
bio(16).dim=[2,1];
bio(16).sigWidth=2;
bio(16).sigAddress='&p1_standalone_B.FrontWheelSpeeds[0]';
bio(16).ndims=2;
bio(16).size=[];


bio(17).blkName='Hardware I//O/Output/amps to volts';
bio(17).sigName='';
bio(17).portIdx=0;
bio(17).dim=[1,1];
bio(17).sigWidth=1;
bio(17).sigAddress='&p1_standalone_ConstB.ampstovolts';
bio(17).ndims=2;
bio(17).size=[];


bio(18).blkName='Hardware I//O/Output/voltage gain';
bio(18).sigName='';
bio(18).portIdx=0;
bio(18).dim=[2,1];
bio(18).sigWidth=2;
bio(18).sigAddress='&p1_standalone_B.voltagegain[0]';
bio(18).ndims=2;
bio(18).size=[];


bio(19).blkName='Hardware I//O/Output/saturation';
bio(19).sigName='';
bio(19).portIdx=0;
bio(19).dim=[1,1];
bio(19).sigWidth=1;
bio(19).sigAddress='&p1_standalone_B.saturation';
bio(19).ndims=2;
bio(19).size=[];


bio(20).blkName='Hardware I//O/Output/saturation 2';
bio(20).sigName='';
bio(20).portIdx=0;
bio(20).dim=[2,1];
bio(20).sigWidth=2;
bio(20).sigAddress='&p1_standalone_B.saturation2[0]';
bio(20).ndims=2;
bio(20).size=[];


bio(21).blkName='Sensor Data and Estimation/Nonlinear Observer/1//Fnf';
bio(21).sigName='muhat';
bio(21).portIdx=0;
bio(21).dim=[1,1];
bio(21).sigWidth=1;
bio(21).sigAddress='&p1_standalone_B.muhat';
bio(21).ndims=2;
bio(21).size=[];


bio(22).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p1';
bio(22).sigName='ahats';
bio(22).portIdx=0;
bio(22).dim=[2,1];
bio(22).sigWidth=2;
bio(22).sigAddress='&p1_standalone_B.ahats[0]';
bio(22).ndims=2;
bio(22).size=[];


bio(23).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p2';
bio(23).sigName='peakFyf';
bio(23).portIdx=1;
bio(23).dim=[1,1];
bio(23).sigWidth=1;
bio(23).sigAddress='&p1_standalone_B.peakFyf';
bio(23).ndims=2;
bio(23).size=[];


bio(24).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p3';
bio(24).sigName='estimation_flags';
bio(24).portIdx=2;
bio(24).dim=[5,1];
bio(24).sigWidth=5;
bio(24).sigAddress='&p1_standalone_B.estimation_flags[0]';
bio(24).ndims=2;
bio(24).size=[];


bio(25).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p4';
bio(25).sigName='pneumatic_trail';
bio(25).portIdx=3;
bio(25).dim=[2,1];
bio(25).sigWidth=2;
bio(25).sigAddress='&p1_standalone_B.pneumatic_trail[0]';
bio(25).ndims=2;
bio(25).size=[];


bio(26).blkName='Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer/p5';
bio(26).sigName='peak_forces';
bio(26).portIdx=4;
bio(26).dim=[2,1];
bio(26).sigWidth=2;
bio(26).sigAddress='&p1_standalone_B.peak_forces[0]';
bio(26).ndims=2;
bio(26).size=[];


bio(27).blkName='Sensor Data and Estimation/Steering Kinematics/Composite Steer Angle Calculation';
bio(27).sigName='Composite Steer Angle';
bio(27).portIdx=0;
bio(27).dim=[1,1];
bio(27).sigWidth=1;
bio(27).sigAddress='&p1_standalone_B.CompositeSteerAngle';
bio(27).ndims=2;
bio(27).size=[];


bio(28).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain1';
bio(28).sigName='Speed';
bio(28).portIdx=0;
bio(28).dim=[1,1];
bio(28).sigWidth=1;
bio(28).sigAddress='&p1_standalone_B.Speed';
bio(28).ndims=2;
bio(28).size=[];


bio(29).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain2';
bio(29).sigName='';
bio(29).portIdx=0;
bio(29).dim=[1,1];
bio(29).sigWidth=1;
bio(29).sigAddress='&p1_standalone_B.Gain2';
bio(29).ndims=2;
bio(29).size=[];


bio(30).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain3';
bio(30).sigName='Speed';
bio(30).portIdx=0;
bio(30).dim=[1,1];
bio(30).sigWidth=1;
bio(30).sigAddress='&p1_standalone_B.Speed_b';
bio(30).ndims=2;
bio(30).size=[];


bio(31).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain4';
bio(31).sigName='';
bio(31).portIdx=0;
bio(31).dim=[1,1];
bio(31).sigWidth=1;
bio(31).sigAddress='&p1_standalone_B.Gain4';
bio(31).ndims=2;
bio(31).size=[];


bio(32).blkName='Sensor Data and Estimation/Vehicle State Estimator/Gain5';
bio(32).sigName='';
bio(32).portIdx=0;
bio(32).dim=[1,1];
bio(32).sigWidth=1;
bio(32).sigAddress='&p1_standalone_B.Gain5';
bio(32).ndims=2;
bio(32).size=[];


bio(33).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter';
bio(33).sigName='Front Left';
bio(33).portIdx=0;
bio(33).dim=[1,1];
bio(33).sigWidth=1;
bio(33).sigAddress='&p1_standalone_B.FrontLeft';
bio(33).ndims=2;
bio(33).size=[];


bio(34).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter1';
bio(34).sigName='Horiz Speed';
bio(34).portIdx=0;
bio(34).dim=[1,1];
bio(34).sigWidth=1;
bio(34).sigAddress='&p1_standalone_B.HorizSpeed';
bio(34).ndims=2;
bio(34).size=[];


bio(35).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter2';
bio(35).sigName='Front Right';
bio(35).portIdx=0;
bio(35).dim=[1,1];
bio(35).sigWidth=1;
bio(35).sigAddress='&p1_standalone_B.FrontRight';
bio(35).ndims=2;
bio(35).size=[];


bio(36).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter3';
bio(36).sigName='Rear Left';
bio(36).portIdx=0;
bio(36).dim=[1,1];
bio(36).sigWidth=1;
bio(36).sigAddress='&p1_standalone_B.RearLeft';
bio(36).ndims=2;
bio(36).size=[];


bio(37).blkName='Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter4';
bio(37).sigName='Rear Right';
bio(37).portIdx=0;
bio(37).dim=[1,1];
bio(37).sigWidth=1;
bio(37).sigAddress='&p1_standalone_B.RearRight';
bio(37).ndims=2;
bio(37).size=[];


bio(38).blkName='User Controllers/Steering Controller/Sum';
bio(38).sigName='';
bio(38).portIdx=0;
bio(38).dim=[1,1];
bio(38).sigWidth=1;
bio(38).sigAddress='&p1_standalone_B.Sum_gj';
bio(38).ndims=2;
bio(38).size=[];


bio(39).blkName='User Controllers/Steering Controller/Heavy Filter';
bio(39).sigName='';
bio(39).portIdx=0;
bio(39).dim=[1,1];
bio(39).sigWidth=1;
bio(39).sigAddress='&p1_standalone_B.HeavyFilter';
bio(39).ndims=2;
bio(39).size=[];


bio(40).blkName='User Controllers/Steering Controller/Nominal Filter';
bio(40).sigName='';
bio(40).portIdx=0;
bio(40).dim=[1,1];
bio(40).sigWidth=1;
bio(40).sigAddress='&p1_standalone_B.NominalFilter';
bio(40).ndims=2;
bio(40).size=[];


bio(41).blkName='Controllers and Output/Steering Controller/LF steering controller/Convert to output shaft angle';
bio(41).sigName='';
bio(41).portIdx=0;
bio(41).dim=[1,1];
bio(41).sigWidth=1;
bio(41).sigAddress='&p1_standalone_B.Converttooutputshaftangle';
bio(41).ndims=2;
bio(41).size=[];


bio(42).blkName='Controllers and Output/Steering Controller/LF steering controller/Convert to output shaft angle (too)';
bio(42).sigName='';
bio(42).portIdx=0;
bio(42).dim=[1,1];
bio(42).sigWidth=1;
bio(42).sigAddress='&p1_standalone_B.Converttooutputshaftangletoo';
bio(42).ndims=2;
bio(42).size=[];


bio(43).blkName='Controllers and Output/Steering Controller/LF steering controller/Kd';
bio(43).sigName='';
bio(43).portIdx=0;
bio(43).dim=[1,1];
bio(43).sigWidth=1;
bio(43).sigAddress='&p1_standalone_B.Kd';
bio(43).ndims=2;
bio(43).size=[];


bio(44).blkName='Controllers and Output/Steering Controller/LF steering controller/Ki';
bio(44).sigName='';
bio(44).portIdx=0;
bio(44).dim=[1,1];
bio(44).sigWidth=1;
bio(44).sigAddress='&p1_standalone_B.Ki';
bio(44).ndims=2;
bio(44).size=[];


bio(45).blkName='Controllers and Output/Steering Controller/LF steering controller/Sum1';
bio(45).sigName='';
bio(45).portIdx=0;
bio(45).dim=[1,1];
bio(45).sigWidth=1;
bio(45).sigAddress='&p1_standalone_B.Sum1_l';
bio(45).ndims=2;
bio(45).size=[];


bio(46).blkName='Controllers and Output/Steering Controller/LF steering controller/Sum2';
bio(46).sigName='';
bio(46).portIdx=0;
bio(46).dim=[1,1];
bio(46).sigWidth=1;
bio(46).sigAddress='&p1_standalone_B.Sum2_l';
bio(46).ndims=2;
bio(46).size=[];


bio(47).blkName='Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter 2';
bio(47).sigName='';
bio(47).portIdx=0;
bio(47).dim=[1,1];
bio(47).sigWidth=1;
bio(47).sigAddress='&p1_standalone_B.DerivativeLowpassfilter2';
bio(47).ndims=2;
bio(47).size=[];


bio(48).blkName='Controllers and Output/Steering Controller/RF steering controller/Convert to output shaft angle';
bio(48).sigName='';
bio(48).portIdx=0;
bio(48).dim=[1,1];
bio(48).sigWidth=1;
bio(48).sigAddress='&p1_standalone_B.Converttooutputshaftangle_n';
bio(48).ndims=2;
bio(48).size=[];


bio(49).blkName='Controllers and Output/Steering Controller/RF steering controller/Convert to output shaft angle (too)';
bio(49).sigName='';
bio(49).portIdx=0;
bio(49).dim=[1,1];
bio(49).sigWidth=1;
bio(49).sigAddress='&p1_standalone_B.Converttooutputshaftangletoo_e';
bio(49).ndims=2;
bio(49).size=[];


bio(50).blkName='Controllers and Output/Steering Controller/RF steering controller/Kd';
bio(50).sigName='';
bio(50).portIdx=0;
bio(50).dim=[1,1];
bio(50).sigWidth=1;
bio(50).sigAddress='&p1_standalone_B.Kd_j';
bio(50).ndims=2;
bio(50).size=[];


bio(51).blkName='Controllers and Output/Steering Controller/RF steering controller/Ki';
bio(51).sigName='';
bio(51).portIdx=0;
bio(51).dim=[1,1];
bio(51).sigWidth=1;
bio(51).sigAddress='&p1_standalone_B.Ki_o';
bio(51).ndims=2;
bio(51).size=[];


bio(52).blkName='Controllers and Output/Steering Controller/RF steering controller/Sum1';
bio(52).sigName='';
bio(52).portIdx=0;
bio(52).dim=[1,1];
bio(52).sigWidth=1;
bio(52).sigAddress='&p1_standalone_B.Sum1_f';
bio(52).ndims=2;
bio(52).size=[];


bio(53).blkName='Controllers and Output/Steering Controller/RF steering controller/Sum2';
bio(53).sigName='';
bio(53).portIdx=0;
bio(53).dim=[1,1];
bio(53).sigWidth=1;
bio(53).sigAddress='&p1_standalone_B.Sum2_o';
bio(53).ndims=2;
bio(53).size=[];


bio(54).blkName='Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter 2';
bio(54).sigName='';
bio(54).portIdx=0;
bio(54).dim=[1,1];
bio(54).sigWidth=1;
bio(54).sigAddress='&p1_standalone_B.DerivativeLowpassfilter2_d';
bio(54).ndims=2;
bio(54).size=[];


bio(55).blkName='Controllers and Output/Steering Controller/standby/Enable Controller';
bio(55).sigName='';
bio(55).portIdx=0;
bio(55).dim=[1,1];
bio(55).sigWidth=1;
bio(55).sigAddress='&p1_standalone_B.EnableController';
bio(55).ndims=2;
bio(55).size=[];


bio(56).blkName='Controllers and Output/Steering Controller/standby/Derivative & Low-pass filter';
bio(56).sigName='';
bio(56).portIdx=0;
bio(56).dim=[1,1];
bio(56).sigWidth=1;
bio(56).sigAddress='&p1_standalone_B.DerivativeLowpassfilter';
bio(56).ndims=2;
bio(56).size=[];


bio(57).blkName='Controllers and Output/Steering Controller/standby/filter';
bio(57).sigName='';
bio(57).portIdx=0;
bio(57).dim=[1,1];
bio(57).sigWidth=1;
bio(57).sigAddress='&p1_standalone_B.filter';
bio(57).ndims=2;
bio(57).size=[];


bio(58).blkName='Controllers and Output/Steering Controller/standby/filter1';
bio(58).sigName='';
bio(58).portIdx=0;
bio(58).dim=[1,1];
bio(58).sigWidth=1;
bio(58).sigAddress='&p1_standalone_B.filter1';
bio(58).ndims=2;
bio(58).size=[];


bio(59).blkName='Hardware I//O/Input/Beeline/Beeline';
bio(59).sigName='';
bio(59).portIdx=0;
bio(59).dim=[10,1];
bio(59).sigWidth=10;
bio(59).sigAddress='&p1_standalone_B.Beeline[0]';
bio(59).ndims=2;
bio(59).size=[];


bio(60).blkName='Hardware I//O/Input/Encoder Board/encoder board /p1';
bio(60).sigName='';
bio(60).portIdx=0;
bio(60).dim=[1,1];
bio(60).sigWidth=1;
bio(60).sigAddress='&p1_standalone_B.encoderboard_o1';
bio(60).ndims=2;
bio(60).size=[];


bio(61).blkName='Hardware I//O/Input/Encoder Board/encoder board /p2';
bio(61).sigName='';
bio(61).portIdx=1;
bio(61).dim=[1,1];
bio(61).sigWidth=1;
bio(61).sigAddress='&p1_standalone_B.encoderboard_o2';
bio(61).ndims=2;
bio(61).size=[];


bio(62).blkName='Hardware I//O/Input/Encoder Board/encoder board /p3';
bio(62).sigName='';
bio(62).portIdx=2;
bio(62).dim=[1,1];
bio(62).sigWidth=1;
bio(62).sigAddress='&p1_standalone_B.encoderboard_o3';
bio(62).ndims=2;
bio(62).size=[];


bio(63).blkName='Hardware I//O/Input/Encoder Board/encoder board /p4';
bio(63).sigName='';
bio(63).portIdx=3;
bio(63).dim=[1,1];
bio(63).sigWidth=1;
bio(63).sigAddress='&p1_standalone_B.encoderboard_o4';
bio(63).ndims=2;
bio(63).size=[];


bio(64).blkName='Hardware I//O/Input/Encoder Board/encoder board /p5';
bio(64).sigName='';
bio(64).portIdx=4;
bio(64).dim=[1,1];
bio(64).sigWidth=1;
bio(64).sigAddress='&p1_standalone_B.encoderboard_o5';
bio(64).ndims=2;
bio(64).size=[];


bio(65).blkName='Hardware I//O/Input/GPS Velocity and Position/GPS Velocity';
bio(65).sigName='';
bio(65).portIdx=0;
bio(65).dim=[19,1];
bio(65).sigWidth=19;
bio(65).sigAddress='&p1_standalone_B.GPSVelocity[0]';
bio(65).ndims=2;
bio(65).size=[];


bio(66).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Gear Ratio';
bio(66).sigName='';
bio(66).portIdx=0;
bio(66).dim=[1,1];
bio(66).sigWidth=1;
bio(66).sigAddress='&p1_standalone_B.HandwheelGearRatio';
bio(66).ndims=2;
bio(66).size=[];


bio(67).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Harmonic Drive Gear Ratio';
bio(67).sigName='';
bio(67).portIdx=0;
bio(67).dim=[1,1];
bio(67).sigWidth=1;
bio(67).sigAddress='&p1_standalone_B.HarmonicDriveGearRatio';
bio(67).ndims=2;
bio(67).size=[];


bio(68).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Harmonic Drive Gear Ratio1';
bio(68).sigName='';
bio(68).portIdx=0;
bio(68).dim=[1,1];
bio(68).sigWidth=1;
bio(68).sigAddress='&p1_standalone_B.HarmonicDriveGearRatio1';
bio(68).ndims=2;
bio(68).size=[];


bio(69).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/encoder scaling';
bio(69).sigName='Primary';
bio(69).portIdx=0;
bio(69).dim=[1,1];
bio(69).sigWidth=1;
bio(69).sigAddress='&p1_standalone_B.Primary';
bio(69).ndims=2;
bio(69).size=[];


bio(70).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Sum';
bio(70).sigName='';
bio(70).portIdx=0;
bio(70).dim=[1,1];
bio(70).sigWidth=1;
bio(70).sigAddress='&p1_standalone_B.Sum';
bio(70).ndims=2;
bio(70).size=[];


bio(71).blkName='Hardware I//O/Input/INS scaling/deg to rad';
bio(71).sigName='yaw';
bio(71).portIdx=0;
bio(71).dim=[1,1];
bio(71).sigWidth=1;
bio(71).sigAddress='&p1_standalone_B.yaw';
bio(71).ndims=2;
bio(71).size=[];


bio(72).blkName='Hardware I//O/Input/Rear Wheel Speed Processing/de-quadrature';
bio(72).sigName='';
bio(72).portIdx=0;
bio(72).dim=[2,1];
bio(72).sigWidth=2;
bio(72).sigAddress='&p1_standalone_B.dequadrature[0]';
bio(72).ndims=2;
bio(72).size=[];


bio(73).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/gearbox ratio';
bio(73).sigName='';
bio(73).portIdx=0;
bio(73).dim=[2,1];
bio(73).sigWidth=2;
bio(73).sigAddress='&p1_standalone_B.gearboxratio[0]';
bio(73).ndims=2;
bio(73).size=[];


bio(74).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Sum1';
bio(74).sigName='';
bio(74).portIdx=0;
bio(74).dim=[2,1];
bio(74).sigWidth=2;
bio(74).sigAddress='&p1_standalone_B.Sum1_g[0]';
bio(74).ndims=2;
bio(74).size=[];


bio(75).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Sum2';
bio(75).sigName='';
bio(75).portIdx=0;
bio(75).dim=[2,1];
bio(75).sigWidth=2;
bio(75).sigAddress='&p1_standalone_B.Sum2_g[0]';
bio(75).ndims=2;
bio(75).size=[];


bio(76).blkName='Hardware I//O/Input/Subsystem1/Sum';
bio(76).sigName='';
bio(76).portIdx=0;
bio(76).dim=[1,1];
bio(76).sigWidth=1;
bio(76).sigAddress='&p1_standalone_B.Sum_g';
bio(76).ndims=2;
bio(76).size=[];


bio(77).blkName='Hardware I//O/Input/Subsystem2/Sum';
bio(77).sigName='';
bio(77).portIdx=0;
bio(77).dim=[1,1];
bio(77).sigWidth=1;
bio(77).sigAddress='&p1_standalone_B.Sum_l';
bio(77).ndims=2;
bio(77).size=[];


bio(78).blkName='Hardware I//O/Input/current scaling/Sum';
bio(78).sigName='';
bio(78).portIdx=0;
bio(78).dim=[2,1];
bio(78).sigWidth=2;
bio(78).sigAddress='&p1_standalone_B.Sum_h[0]';
bio(78).ndims=2;
bio(78).size=[];


bio(79).blkName='Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Left Normal Load';
bio(79).sigName='Left';
bio(79).portIdx=0;
bio(79).dim=[1,1];
bio(79).sigWidth=1;
bio(79).sigAddress='&p1_standalone_B.Left_d';
bio(79).ndims=2;
bio(79).size=[];


bio(80).blkName='Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Right Normal Load';
bio(80).sigName='Right';
bio(80).portIdx=0;
bio(80).dim=[1,1];
bio(80).sigWidth=1;
bio(80).sigAddress='&p1_standalone_B.Right_o';
bio(80).ndims=2;
bio(80).size=[];


bio(81).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable';
bio(81).sigName='';
bio(81).portIdx=0;
bio(81).dim=[1,1];
bio(81).sigWidth=1;
bio(81).sigAddress='&p1_standalone_B.Disable';
bio(81).ndims=2;
bio(81).size=[];


bio(82).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable1';
bio(82).sigName='';
bio(82).portIdx=0;
bio(82).dim=[1,1];
bio(82).sigWidth=1;
bio(82).sigAddress='&p1_standalone_B.Disable1';
bio(82).ndims=2;
bio(82).size=[];


bio(83).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable3';
bio(83).sigName='';
bio(83).portIdx=0;
bio(83).dim=[1,1];
bio(83).sigWidth=1;
bio(83).sigAddress='&p1_standalone_B.Disable3';
bio(83).ndims=2;
bio(83).size=[];


bio(84).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable4';
bio(84).sigName='';
bio(84).portIdx=0;
bio(84).dim=[1,1];
bio(84).sigWidth=1;
bio(84).sigAddress='&p1_standalone_B.Disable4';
bio(84).ndims=2;
bio(84).size=[];


bio(85).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable6';
bio(85).sigName='Left';
bio(85).portIdx=0;
bio(85).dim=[1,1];
bio(85).sigWidth=1;
bio(85).sigAddress='&p1_standalone_B.Left_p';
bio(85).ndims=2;
bio(85).size=[];


bio(86).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable7';
bio(86).sigName='Right';
bio(86).portIdx=0;
bio(86).dim=[1,1];
bio(86).sigWidth=1;
bio(86).sigAddress='&p1_standalone_B.Right_b';
bio(86).ndims=2;
bio(86).size=[];


bio(87).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Damping';
bio(87).sigName='Left';
bio(87).portIdx=0;
bio(87).dim=[1,1];
bio(87).sigWidth=1;
bio(87).sigAddress='&p1_standalone_B.Left_l';
bio(87).ndims=2;
bio(87).size=[];


bio(88).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Damping1';
bio(88).sigName='Right';
bio(88).portIdx=0;
bio(88).dim=[1,1];
bio(88).sigWidth=1;
bio(88).sigAddress='&p1_standalone_B.Right_e';
bio(88).ndims=2;
bio(88).size=[];


bio(89).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Friction';
bio(89).sigName='';
bio(89).portIdx=0;
bio(89).dim=[1,1];
bio(89).sigWidth=1;
bio(89).sigAddress='&p1_standalone_B.WheelFriction';
bio(89).ndims=2;
bio(89).size=[];


bio(90).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Friction1';
bio(90).sigName='';
bio(90).portIdx=0;
bio(90).dim=[1,1];
bio(90).sigWidth=1;
bio(90).sigAddress='&p1_standalone_B.WheelFriction1';
bio(90).ndims=2;
bio(90).size=[];


bio(91).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Inertia';
bio(91).sigName='Left';
bio(91).portIdx=0;
bio(91).dim=[1,1];
bio(91).sigWidth=1;
bio(91).sigAddress='&p1_standalone_B.Left_e';
bio(91).ndims=2;
bio(91).size=[];


bio(92).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Inertia1';
bio(92).sigName='Right';
bio(92).portIdx=0;
bio(92).dim=[1,1];
bio(92).sigWidth=1;
bio(92).sigAddress='&p1_standalone_B.Right_h';
bio(92).ndims=2;
bio(92).size=[];


bio(93).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Product';
bio(93).sigName='Left';
bio(93).portIdx=0;
bio(93).dim=[1,1];
bio(93).sigWidth=1;
bio(93).sigAddress='&p1_standalone_B.Left_k';
bio(93).ndims=2;
bio(93).size=[];


bio(94).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Product1';
bio(94).sigName='Right';
bio(94).portIdx=0;
bio(94).dim=[1,1];
bio(94).sigWidth=1;
bio(94).sigAddress='&p1_standalone_B.Right_k';
bio(94).ndims=2;
bio(94).size=[];


bio(95).blkName='Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Left Arm Length Look-up';
bio(95).sigName='';
bio(95).portIdx=0;
bio(95).dim=[1,1];
bio(95).sigWidth=1;
bio(95).sigAddress='&p1_standalone_B.LeftArmLengthLookup';
bio(95).ndims=2;
bio(95).size=[];


bio(96).blkName='Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Right Arm Length Look-up';
bio(96).sigName='';
bio(96).portIdx=0;
bio(96).dim=[1,1];
bio(96).sigWidth=1;
bio(96).sigAddress='&p1_standalone_B.RightArmLengthLookup';
bio(96).ndims=2;
bio(96).size=[];


bio(97).blkName='Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Product';
bio(97).sigName='Left';
bio(97).portIdx=0;
bio(97).dim=[1,1];
bio(97).sigWidth=1;
bio(97).sigAddress='&p1_standalone_B.Left_o';
bio(97).ndims=2;
bio(97).size=[];


bio(98).blkName='Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Product1';
bio(98).sigName='Right';
bio(98).portIdx=0;
bio(98).dim=[1,1];
bio(98).sigWidth=1;
bio(98).sigAddress='&p1_standalone_B.Right_p';
bio(98).ndims=2;
bio(98).size=[];


bio(99).blkName='Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Left Steer Angle Look-up';
bio(99).sigName='Left';
bio(99).portIdx=0;
bio(99).dim=[1,1];
bio(99).sigWidth=1;
bio(99).sigAddress='&p1_standalone_B.Left';
bio(99).ndims=2;
bio(99).size=[];


bio(100).blkName='Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Right Steer Angle Look-up';
bio(100).sigName='Right';
bio(100).portIdx=0;
bio(100).dim=[1,1];
bio(100).sigWidth=1;
bio(100).sigAddress='&p1_standalone_B.Right';
bio(100).ndims=2;
bio(100).size=[];


bio(101).blkName='Sensor Data and Estimation/Vehicle State Estimator/Select: Yaw Rate/Gain';
bio(101).sigName='';
bio(101).portIdx=0;
bio(101).dim=[1,1];
bio(101).sigWidth=1;
bio(101).sigAddress='&p1_standalone_B.Gain_a';
bio(101).ndims=2;
bio(101).size=[];


bio(102).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad1';
bio(102).sigName='Yaw Rate';
bio(102).portIdx=0;
bio(102).dim=[1,1];
bio(102).sigWidth=1;
bio(102).sigAddress='&p1_standalone_B.YawRate';
bio(102).ndims=2;
bio(102).size=[];


bio(103).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad2';
bio(103).sigName='Yaw Angle';
bio(103).portIdx=0;
bio(103).dim=[1,1];
bio(103).sigWidth=1;
bio(103).sigAddress='&p1_standalone_B.YawAngle';
bio(103).ndims=2;
bio(103).size=[];


bio(104).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad3';
bio(104).sigName='Roll Angle';
bio(104).portIdx=0;
bio(104).dim=[1,1];
bio(104).sigWidth=1;
bio(104).sigAddress='&p1_standalone_B.RollAngle';
bio(104).ndims=2;
bio(104).size=[];


bio(105).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad4';
bio(105).sigName='Roll Rate';
bio(105).portIdx=0;
bio(105).dim=[1,1];
bio(105).sigWidth=1;
bio(105).sigAddress='&p1_standalone_B.RollRate';
bio(105).ndims=2;
bio(105).size=[];


bio(106).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad5';
bio(106).sigName='Yaw Gyro Bias';
bio(106).portIdx=0;
bio(106).dim=[1,1];
bio(106).sigWidth=1;
bio(106).sigAddress='&p1_standalone_B.YawGyroBias';
bio(106).ndims=2;
bio(106).size=[];


bio(107).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad6';
bio(107).sigName='Roll Gyro Bias';
bio(107).portIdx=0;
bio(107).dim=[1,1];
bio(107).sigWidth=1;
bio(107).sigAddress='&p1_standalone_B.RollGyroBias';
bio(107).ndims=2;
bio(107).size=[];


bio(108).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad7';
bio(108).sigName='Sideslip';
bio(108).portIdx=0;
bio(108).dim=[1,1];
bio(108).sigWidth=1;
bio(108).sigAddress='&p1_standalone_B.Sideslip';
bio(108).ndims=2;
bio(108).size=[];


bio(109).blkName='User Controllers/Steering Controller/HAL-9000/Clock';
bio(109).sigName='';
bio(109).portIdx=0;
bio(109).dim=[1,1];
bio(109).sigWidth=1;
bio(109).sigAddress='&p1_standalone_B.Clock';
bio(109).ndims=2;
bio(109).size=[];


bio(110).blkName='User Controllers/Steering Controller/HAL-9000/Product';
bio(110).sigName='';
bio(110).portIdx=0;
bio(110).dim=[1,1];
bio(110).sigWidth=1;
bio(110).sigAddress='&p1_standalone_B.Product';
bio(110).ndims=2;
bio(110).size=[];


bio(111).blkName='User Controllers/Steering Controller/HAL-9000/Product1';
bio(111).sigName='';
bio(111).portIdx=0;
bio(111).dim=[1,1];
bio(111).sigWidth=1;
bio(111).sigAddress='&p1_standalone_B.Product1';
bio(111).ndims=2;
bio(111).size=[];


bio(112).blkName='User Controllers/Steering Controller/HAL-9000/Sum';
bio(112).sigName='';
bio(112).portIdx=0;
bio(112).dim=[1,1];
bio(112).sigWidth=1;
bio(112).sigAddress='&p1_standalone_B.Sum_hm';
bio(112).ndims=2;
bio(112).size=[];


bio(113).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Encoder Initialization/Encoder Initializer';
bio(113).sigName='';
bio(113).portIdx=0;
bio(113).dim=[1,1];
bio(113).sigWidth=1;
bio(113).sigAddress='&p1_standalone_B.EncoderInitializer';
bio(113).ndims=2;
bio(113).size=[];


bio(114).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Encoder Initialization/Unwrapper';
bio(114).sigName='';
bio(114).portIdx=0;
bio(114).dim=[1,1];
bio(114).sigWidth=1;
bio(114).sigAddress='&p1_standalone_B.Unwrapper';
bio(114).ndims=2;
bio(114).size=[];


bio(115).blkName='Hardware I//O/Input/OEM4 wgsxyz2enu and status conversion/position preprocessing/Sum';
bio(115).sigName='';
bio(115).portIdx=0;
bio(115).dim=[1,1];
bio(115).sigWidth=1;
bio(115).sigAddress='&p1_standalone_B.Sum_b';
bio(115).ndims=2;
bio(115).size=[];


bio(116).blkName='Hardware I//O/Input/OEM4 wgsxyz2enu and status conversion/position preprocessing/Sum1';
bio(116).sigName='';
bio(116).portIdx=0;
bio(116).dim=[1,1];
bio(116).sigWidth=1;
bio(116).sigAddress='&p1_standalone_B.Sum1';
bio(116).ndims=2;
bio(116).size=[];


bio(117).blkName='Hardware I//O/Input/OEM4 wgsxyz2enu and status conversion/position preprocessing/Sum2';
bio(117).sigName='';
bio(117).portIdx=0;
bio(117).dim=[1,1];
bio(117).sigWidth=1;
bio(117).sigAddress='&p1_standalone_B.Sum2';
bio(117).ndims=2;
bio(117).size=[];


bio(118).blkName='Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc/Unwrapping';
bio(118).sigName='';
bio(118).portIdx=0;
bio(118).dim=[1,1];
bio(118).sigWidth=1;
bio(118).sigAddress='&p1_standalone_B.Unwrapping';
bio(118).ndims=2;
bio(118).size=[];


bio(119).blkName='Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc1/Unwrapping';
bio(119).sigName='';
bio(119).portIdx=0;
bio(119).dim=[1,1];
bio(119).sigWidth=1;
bio(119).sigAddress='&p1_standalone_B.Unwrapping_n';
bio(119).ndims=2;
bio(119).size=[];


bio(120).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Left/Encoder Initializer';
bio(120).sigName='';
bio(120).portIdx=0;
bio(120).dim=[1,1];
bio(120).sigWidth=1;
bio(120).sigAddress='&p1_standalone_B.EncoderInitializer_i';
bio(120).ndims=2;
bio(120).size=[];


bio(121).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Left/Unwrapper';
bio(121).sigName='';
bio(121).portIdx=0;
bio(121).dim=[1,1];
bio(121).sigWidth=1;
bio(121).sigAddress='&p1_standalone_B.Unwrapper_o';
bio(121).ndims=2;
bio(121).size=[];


bio(122).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Right/Encoder Initializer';
bio(122).sigName='';
bio(122).portIdx=0;
bio(122).dim=[1,1];
bio(122).sigWidth=1;
bio(122).sigAddress='&p1_standalone_B.EncoderInitializer_k';
bio(122).ndims=2;
bio(122).size=[];


bio(123).blkName='Hardware I//O/Input/Steering Motor Encoder Initialization/Right/Unwrapper';
bio(123).sigName='';
bio(123).portIdx=0;
bio(123).dim=[1,1];
bio(123).sigWidth=1;
bio(123).sigAddress='&p1_standalone_B.Unwrapper_a';
bio(123).ndims=2;
bio(123).size=[];


bio(124).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Left Steer Angle Look-up1';
bio(124).sigName='Left';
bio(124).portIdx=0;
bio(124).dim=[1,1];
bio(124).sigWidth=1;
bio(124).sigAddress='&p1_standalone_B.Left_j';
bio(124).ndims=2;
bio(124).size=[];


bio(125).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Right Steer Angle Look-up1';
bio(125).sigName='Right';
bio(125).portIdx=0;
bio(125).dim=[1,1];
bio(125).sigWidth=1;
bio(125).sigAddress='&p1_standalone_B.Right_h0';
bio(125).ndims=2;
bio(125).size=[];


bio(126).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations/Diff';
bio(126).sigName='';
bio(126).portIdx=0;
bio(126).dim=[1,1];
bio(126).sigWidth=1;
bio(126).sigAddress='&p1_standalone_B.Diff';
bio(126).ndims=2;
bio(126).size=[];


bio(127).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations/TSamp';
bio(127).sigName='';
bio(127).portIdx=0;
bio(127).dim=[1,1];
bio(127).sigWidth=1;
bio(127).sigAddress='&p1_standalone_B.TSamp';
bio(127).ndims=2;
bio(127).size=[];


bio(128).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations1/Diff';
bio(128).sigName='';
bio(128).portIdx=0;
bio(128).dim=[1,1];
bio(128).sigWidth=1;
bio(128).sigAddress='&p1_standalone_B.Diff_b';
bio(128).ndims=2;
bio(128).size=[];


bio(129).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations1/TSamp';
bio(129).sigName='';
bio(129).portIdx=0;
bio(129).dim=[1,1];
bio(129).sigWidth=1;
bio(129).sigAddress='&p1_standalone_B.TSamp_i';
bio(129).ndims=2;
bio(129).size=[];


bio(130).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities/Diff';
bio(130).sigName='';
bio(130).portIdx=0;
bio(130).dim=[1,1];
bio(130).sigWidth=1;
bio(130).sigAddress='&p1_standalone_B.Diff_o';
bio(130).ndims=2;
bio(130).size=[];


bio(131).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities/TSamp';
bio(131).sigName='';
bio(131).portIdx=0;
bio(131).dim=[1,1];
bio(131).sigWidth=1;
bio(131).sigAddress='&p1_standalone_B.TSamp_a';
bio(131).ndims=2;
bio(131).size=[];


bio(132).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities1/Diff';
bio(132).sigName='';
bio(132).portIdx=0;
bio(132).dim=[1,1];
bio(132).sigWidth=1;
bio(132).sigAddress='&p1_standalone_B.Diff_ok';
bio(132).ndims=2;
bio(132).size=[];


bio(133).blkName='Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities1/TSamp';
bio(133).sigName='';
bio(133).portIdx=0;
bio(133).dim=[1,1];
bio(133).sigWidth=1;
bio(133).sigAddress='&p1_standalone_B.TSamp_h';
bio(133).ndims=2;
bio(133).size=[];


bio(134).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Gain to make CCW';
bio(134).sigName='';
bio(134).portIdx=0;
bio(134).dim=[1,1];
bio(134).sigWidth=1;
bio(134).sigAddress='&p1_standalone_B.GaintomakeCCW';
bio(134).ndims=2;
bio(134).size=[];


bio(135).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Position Filter';
bio(135).sigName='';
bio(135).portIdx=0;
bio(135).dim=[42,1];
bio(135).sigWidth=42;
bio(135).sigAddress='&p1_standalone_B.PositionFilter[0]';
bio(135).ndims=2;
bio(135).size=[];


bio(136).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
bio(136).sigName='';
bio(136).portIdx=0;
bio(136).dim=[44,1];
bio(136).sigWidth=44;
bio(136).sigAddress='&p1_standalone_B.SuperFilter[0]';
bio(136).ndims=2;
bio(136).size=[];


bio(137).blkName='User Controllers/Steering Controller/HAL-9000/Latch/In1';
bio(137).sigName='';
bio(137).portIdx=0;
bio(137).dim=[1,1];
bio(137).sigWidth=1;
bio(137).sigAddress='&p1_standalone_B.In1';
bio(137).ndims=2;
bio(137).size=[];


bio(138).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Logical Operator4';
bio(138).sigName='';
bio(138).portIdx=0;
bio(138).dim=[1,1];
bio(138).sigWidth=1;
bio(138).sigAddress='&p1_standalone_B.LogicalOperator4';
bio(138).ndims=2;
bio(138).size=[];


bio(139).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Memory3';
bio(139).sigName='';
bio(139).portIdx=0;
bio(139).dim=[1,1];
bio(139).sigWidth=1;
bio(139).sigAddress='&p1_standalone_B.Memory3';
bio(139).ndims=2;
bio(139).size=[];


bio(140).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Enable Timer/Relational Operator';
bio(140).sigName='';
bio(140).portIdx=0;
bio(140).dim=[1,1];
bio(140).sigWidth=1;
bio(140).sigAddress='&p1_standalone_B.RelationalOperator';
bio(140).ndims=2;
bio(140).size=[];


bio(141).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Memory2';
bio(141).sigName='';
bio(141).portIdx=0;
bio(141).dim=[1,1];
bio(141).sigWidth=1;
bio(141).sigAddress='&p1_standalone_B.Memory2';
bio(141).ndims=2;
bio(141).size=[];


bio(142).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Switch Over/Switch';
bio(142).sigName='';
bio(142).portIdx=0;
bio(142).dim=[1,1];
bio(142).sigWidth=1;
bio(142).sigAddress='&p1_standalone_B.Switch';
bio(142).ndims=2;
bio(142).size=[];


bio(143).blkName='Hardware I//O/Input/OEM4 wgsxyz2enu and status conversion/position preprocessing/Position Converter/S-Function2';
bio(143).sigName='';
bio(143).portIdx=0;
bio(143).dim=[3,1];
bio(143).sigWidth=3;
bio(143).sigAddress='&p1_standalone_B.SFunction2[0]';
bio(143).ndims=2;
bio(143).size=[];


bio(144).blkName='Hardware I//O/Input/OEM4 wgsxyz2enu and status conversion/velocity preprocessing/Velocity Converter/S-Function2';
bio(144).sigName='';
bio(144).portIdx=0;
bio(144).dim=[3,1];
bio(144).sigWidth=3;
bio(144).sigAddress='&p1_standalone_B.SFunction2_b[0]';
bio(144).ndims=2;
bio(144).size=[];


bio(145).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/cross coupling from ax';
bio(145).sigName='';
bio(145).portIdx=0;
bio(145).dim=[1,1];
bio(145).sigWidth=1;
bio(145).sigAddress='&p1_standalone_B.crosscouplingfromax';
bio(145).ndims=2;
bio(145).size=[];


bio(146).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/Sum4';
bio(146).sigName='';
bio(146).portIdx=0;
bio(146).dim=[1,1];
bio(146).sigWidth=1;
bio(146).sigAddress='&p1_standalone_B.Sum4';
bio(146).ndims=2;
bio(146).size=[];


bio(147).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/Sum5';
bio(147).sigName='';
bio(147).portIdx=0;
bio(147).dim=[1,1];
bio(147).sigWidth=1;
bio(147).sigAddress='&p1_standalone_B.Sum5_p';
bio(147).ndims=2;
bio(147).size=[];


bio(148).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/Gain';
bio(148).sigName='';
bio(148).portIdx=0;
bio(148).dim=[1,1];
bio(148).sigWidth=1;
bio(148).sigAddress='&p1_standalone_B.Gain';
bio(148).ndims=2;
bio(148).size=[];


bio(149).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/Delay function';
bio(149).sigName='';
bio(149).portIdx=0;
bio(149).dim=[1,1];
bio(149).sigWidth=1;
bio(149).sigAddress='&p1_standalone_B.Delayfunction';
bio(149).ndims=2;
bio(149).size=[];


bio(150).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/S-Function4';
bio(150).sigName='';
bio(150).portIdx=0;
bio(150).dim=[1,1];
bio(150).sigWidth=1;
bio(150).sigAddress='&p1_standalone_B.SFunction4';
bio(150).ndims=2;
bio(150).size=[];


bio(151).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/Sum1';
bio(151).sigName='';
bio(151).portIdx=0;
bio(151).dim=[1,1];
bio(151).sigWidth=1;
bio(151).sigAddress='&p1_standalone_B.Sum1_b';
bio(151).ndims=2;
bio(151).size=[];


bio(152).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Beeline GPS Pre-Processing/Switch3.5';
bio(152).sigName='';
bio(152).portIdx=0;
bio(152).dim=[1,1];
bio(152).sigWidth=1;
bio(152).sigAddress='&p1_standalone_B.Switch35';
bio(152).ndims=2;
bio(152).size=[];


bio(153).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/OEM4 GPS Pre-Processing/Comparison to .1';
bio(153).sigName='';
bio(153).portIdx=0;
bio(153).dim=[1,1];
bio(153).sigWidth=1;
bio(153).sigAddress='&p1_standalone_B.Comparisonto1';
bio(153).ndims=2;
bio(153).size=[];


bio(154).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/OEM4 GPS Pre-Processing/Delay function';
bio(154).sigName='';
bio(154).portIdx=0;
bio(154).dim=[1,1];
bio(154).sigWidth=1;
bio(154).sigAddress='&p1_standalone_B.Delayfunction_p';
bio(154).ndims=2;
bio(154).size=[];


bio(155).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/OEM4 GPS Pre-Processing/S-Function4';
bio(155).sigName='';
bio(155).portIdx=0;
bio(155).dim=[1,1];
bio(155).sigWidth=1;
bio(155).sigAddress='&p1_standalone_B.SFunction4_i';
bio(155).ndims=2;
bio(155).size=[];


bio(156).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/OEM4 GPS Pre-Processing/Switch2';
bio(156).sigName='';
bio(156).portIdx=0;
bio(156).dim=[1,1];
bio(156).sigWidth=1;
bio(156).sigAddress='&p1_standalone_B.Switch2';
bio(156).ndims=2;
bio(156).size=[];


bio(157).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/cross coupling from roll';
bio(157).sigName='';
bio(157).portIdx=0;
bio(157).dim=[1,1];
bio(157).sigWidth=1;
bio(157).sigAddress='&p1_standalone_B.crosscouplingfromroll';
bio(157).ndims=2;
bio(157).size=[];


bio(158).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/Sum3';
bio(158).sigName='';
bio(158).portIdx=0;
bio(158).dim=[1,1];
bio(158).sigWidth=1;
bio(158).sigAddress='&p1_standalone_B.Sum3';
bio(158).ndims=2;
bio(158).size=[];


bio(159).blkName='Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/Sum5';
bio(159).sigName='';
bio(159).portIdx=0;
bio(159).dim=[1,1];
bio(159).sigWidth=1;
bio(159).sigAddress='&p1_standalone_B.Sum5';
bio(159).ndims=2;
bio(159).size=[];


bio(160).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter/Relational Operator2';
bio(160).sigName='';
bio(160).portIdx=0;
bio(160).dim=[1,1];
bio(160).sigWidth=1;
bio(160).sigAddress='&p1_standalone_B.RelationalOperator2';
bio(160).ndims=2;
bio(160).size=[];


bio(161).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter1/Relational Operator2';
bio(161).sigName='';
bio(161).portIdx=0;
bio(161).dim=[1,1];
bio(161).sigWidth=1;
bio(161).sigAddress='&p1_standalone_B.RelationalOperator2_h';
bio(161).ndims=2;
bio(161).size=[];


bio(162).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Enabled Subsystem/In1';
bio(162).sigName='';
bio(162).portIdx=0;
bio(162).dim=[1,1];
bio(162).sigWidth=1;
bio(162).sigAddress='&p1_standalone_B.EnabledSubsystem_i.In1';
bio(162).ndims=2;
bio(162).size=[];


bio(163).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Enabled Subsystem1/In1';
bio(163).sigName='';
bio(163).portIdx=0;
bio(163).dim=[1,1];
bio(163).sigWidth=1;
bio(163).sigAddress='&p1_standalone_B.EnabledSubsystem1.In1';
bio(163).ndims=2;
bio(163).size=[];


bio(164).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Nearly the Same/Relational Operator';
bio(164).sigName='';
bio(164).portIdx=0;
bio(164).dim=[1,1];
bio(164).sigWidth=1;
bio(164).sigAddress='&p1_standalone_B.RelationalOperator_m';
bio(164).ndims=2;
bio(164).size=[];


bio(165).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Nearly the Same2/Relational Operator';
bio(165).sigName='';
bio(165).portIdx=0;
bio(165).dim=[1,1];
bio(165).sigWidth=1;
bio(165).sigAddress='&p1_standalone_B.RelationalOperator_k';
bio(165).ndims=2;
bio(165).size=[];


bio(166).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Latch/Enabled Subsystem/In1';
bio(166).sigName='';
bio(166).portIdx=0;
bio(166).dim=[1,1];
bio(166).sigWidth=1;
bio(166).sigAddress='&p1_standalone_B.EnabledSubsystem.In1';
bio(166).ndims=2;
bio(166).size=[];


bio(167).blkName='Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Latch1/Enabled Subsystem/In1';
bio(167).sigName='';
bio(167).portIdx=0;
bio(167).dim=[1,1];
bio(167).sigWidth=1;
bio(167).sigAddress='&p1_standalone_B.EnabledSubsystem_h.In1';
bio(167).ndims=2;
bio(167).size=[];


function len = getlenBIO
len = 167;

