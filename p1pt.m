function pt=p1pt
pt = [];
pt(1).blockname = 'Data Logging/Named Mux';
pt(1).paramname = 'P1';
pt(1).class     = 'scalar';
pt(1).nrows     = 1;
pt(1).ncols     = 1;
pt(1).subsource = 'SS_DOUBLE';
pt(1).ndims     = '2';
pt(1).size      = '[]';
pt(getlenPT) = pt(1);

pt(2).blockname = 'Data Logging/Named Mux';
pt(2).paramname = 'P2';
pt(2).class     = 'vector';
pt(2).nrows     = 1;
pt(2).ncols     = 15;
pt(2).subsource = 'SS_DOUBLE';
pt(2).ndims     = '2';
pt(2).size      = '[]';

pt(3).blockname = 'User Controllers/Constant';
pt(3).paramname = 'Value';
pt(3).class     = 'scalar';
pt(3).nrows     = 1;
pt(3).ncols     = 1;
pt(3).subsource = 'SS_DOUBLE';
pt(3).ndims     = '2';
pt(3).size      = '[]';

pt(4).blockname = 'User Controllers/Constant1';
pt(4).paramname = 'Value';
pt(4).class     = 'scalar';
pt(4).nrows     = 1;
pt(4).ncols     = 1;
pt(4).subsource = 'SS_DOUBLE';
pt(4).ndims     = '2';
pt(4).size      = '[]';

pt(5).blockname = 'Controllers and Output/Steering Controller/Constant';
pt(5).paramname = 'Value';
pt(5).class     = 'scalar';
pt(5).nrows     = 1;
pt(5).ncols     = 1;
pt(5).subsource = 'SS_DOUBLE';
pt(5).ndims     = '2';
pt(5).size      = '[]';

pt(6).blockname = 'Controllers and Output/UI Controller/Heartbeat Signal';
pt(6).paramname = 'Amplitude';
pt(6).class     = 'scalar';
pt(6).nrows     = 1;
pt(6).ncols     = 1;
pt(6).subsource = 'SS_DOUBLE';
pt(6).ndims     = '2';
pt(6).size      = '[]';

pt(7).blockname = 'Controllers and Output/UI Controller/Heartbeat Signal';
pt(7).paramname = 'Period';
pt(7).class     = 'scalar';
pt(7).nrows     = 1;
pt(7).ncols     = 1;
pt(7).subsource = 'SS_DOUBLE';
pt(7).ndims     = '2';
pt(7).size      = '[]';

pt(8).blockname = 'Controllers and Output/UI Controller/Heartbeat Signal';
pt(8).paramname = 'PulseWidth';
pt(8).class     = 'scalar';
pt(8).nrows     = 1;
pt(8).ncols     = 1;
pt(8).subsource = 'SS_DOUBLE';
pt(8).ndims     = '2';
pt(8).size      = '[]';

pt(9).blockname = 'Controllers and Output/UI Controller/Heartbeat Signal';
pt(9).paramname = 'PhaseDelay';
pt(9).class     = 'scalar';
pt(9).nrows     = 1;
pt(9).ncols     = 1;
pt(9).subsource = 'SS_DOUBLE';
pt(9).ndims     = '2';
pt(9).size      = '[]';

pt(10).blockname = 'Hardware I//O/Input/alignment (rack 7//16)';
pt(10).paramname = 'Value';
pt(10).class     = 'vector';
pt(10).nrows     = 1;
pt(10).ncols     = 2;
pt(10).subsource = 'SS_DOUBLE';
pt(10).ndims     = '2';
pt(10).size      = '[]';

pt(11).blockname = 'Hardware I//O/Input/Load Cell Scaling';
pt(11).paramname = 'Gain';
pt(11).class     = 'scalar';
pt(11).nrows     = 1;
pt(11).ncols     = 1;
pt(11).subsource = 'SS_DOUBLE';
pt(11).ndims     = '2';
pt(11).size      = '[]';

pt(12).blockname = 'Hardware I//O/Input/Load Cell Scaling Rear';
pt(12).paramname = 'Gain';
pt(12).class     = 'scalar';
pt(12).nrows     = 1;
pt(12).ncols     = 1;
pt(12).subsource = 'SS_DOUBLE';
pt(12).ndims     = '2';
pt(12).size      = '[]';

pt(13).blockname = 'Hardware I//O/Input/encoder scaling';
pt(13).paramname = 'Gain';
pt(13).class     = 'scalar';
pt(13).nrows     = 1;
pt(13).ncols     = 1;
pt(13).subsource = 'SS_DOUBLE';
pt(13).ndims     = '2';
pt(13).size      = '[]';

pt(14).blockname = 'Hardware I//O/Input/voltage scaling';
pt(14).paramname = 'Gain';
pt(14).class     = 'scalar';
pt(14).nrows     = 1;
pt(14).ncols     = 1;
pt(14).subsource = 'SS_DOUBLE';
pt(14).ndims     = '2';
pt(14).size      = '[]';

pt(15).blockname = 'Hardware I//O/Input/Analog Input (DAS)';
pt(15).paramname = 'P1';
pt(15).class     = 'scalar';
pt(15).nrows     = 1;
pt(15).ncols     = 1;
pt(15).subsource = 'SS_DOUBLE';
pt(15).ndims     = '2';
pt(15).size      = '[]';

pt(16).blockname = 'Hardware I//O/Input/Analog Input (DAS)';
pt(16).paramname = 'P2';
pt(16).class     = 'vector';
pt(16).nrows     = 1;
pt(16).ncols     = 16;
pt(16).subsource = 'SS_DOUBLE';
pt(16).ndims     = '2';
pt(16).size      = '[]';

pt(17).blockname = 'Hardware I//O/Input/Analog Input (VSBC)';
pt(17).paramname = 'P1';
pt(17).class     = 'scalar';
pt(17).nrows     = 1;
pt(17).ncols     = 1;
pt(17).subsource = 'SS_DOUBLE';
pt(17).ndims     = '2';
pt(17).size      = '[]';

pt(18).blockname = 'Hardware I//O/Input/Analog Input (VSBC)';
pt(18).paramname = 'P2';
pt(18).class     = 'vector';
pt(18).nrows     = 1;
pt(18).ncols     = 8;
pt(18).subsource = 'SS_DOUBLE';
pt(18).ndims     = '2';
pt(18).size      = '[]';

pt(19).blockname = 'Hardware I//O/Input/Ruby-MM';
pt(19).paramname = 'P1';
pt(19).class     = 'vector';
pt(19).nrows     = 1;
pt(19).ncols     = 4;
pt(19).subsource = 'SS_DOUBLE';
pt(19).ndims     = '2';
pt(19).size      = '[]';

pt(20).blockname = 'Hardware I//O/Input/Ruby-MM';
pt(20).paramname = 'P2';
pt(20).class     = 'scalar';
pt(20).nrows     = 1;
pt(20).ncols     = 1;
pt(20).subsource = 'SS_DOUBLE';
pt(20).ndims     = '2';
pt(20).size      = '[]';

pt(21).blockname = 'Hardware I//O/Input/Ruby-MM';
pt(21).paramname = 'P3';
pt(21).class     = 'vector';
pt(21).nrows     = 1;
pt(21).ncols     = 3;
pt(21).subsource = 'SS_DOUBLE';
pt(21).ndims     = '2';
pt(21).size      = '[]';

pt(22).blockname = 'Hardware I//O/Input/Ruby-MM';
pt(22).paramname = 'P4';
pt(22).class     = 'scalar';
pt(22).nrows     = 1;
pt(22).ncols     = 1;
pt(22).subsource = 'SS_DOUBLE';
pt(22).ndims     = '2';
pt(22).size      = '[]';

pt(23).blockname = 'Hardware I//O/Input/Ruby-MM';
pt(23).paramname = 'P5';
pt(23).class     = 'scalar';
pt(23).nrows     = 1;
pt(23).ncols     = 1;
pt(23).subsource = 'SS_DOUBLE';
pt(23).ndims     = '2';
pt(23).size      = '[]';

pt(24).blockname = 'Hardware I//O/Input/Ruby-MM';
pt(24).paramname = 'P6';
pt(24).class     = 'scalar';
pt(24).nrows     = 1;
pt(24).ncols     = 1;
pt(24).subsource = 'SS_DOUBLE';
pt(24).ndims     = '2';
pt(24).size      = '[]';

pt(25).blockname = 'Hardware I//O/Output/1.0 = full scale';
pt(25).paramname = 'Gain';
pt(25).class     = 'scalar';
pt(25).nrows     = 1;
pt(25).ncols     = 1;
pt(25).subsource = 'SS_DOUBLE';
pt(25).ndims     = '2';
pt(25).size      = '[]';

pt(26).blockname = 'Hardware I//O/Output/amps to volts';
pt(26).paramname = 'Gain';
pt(26).class     = 'scalar';
pt(26).nrows     = 1;
pt(26).ncols     = 1;
pt(26).subsource = 'SS_DOUBLE';
pt(26).ndims     = '2';
pt(26).size      = '[]';

pt(27).blockname = 'Hardware I//O/Output/raw signal';
pt(27).paramname = 'Gain';
pt(27).class     = 'scalar';
pt(27).nrows     = 1;
pt(27).ncols     = 1;
pt(27).subsource = 'SS_DOUBLE';
pt(27).ndims     = '2';
pt(27).size      = '[]';

pt(28).blockname = 'Hardware I//O/Output/voltage gain';
pt(28).paramname = 'Gain';
pt(28).class     = 'scalar';
pt(28).nrows     = 1;
pt(28).ncols     = 1;
pt(28).subsource = 'SS_DOUBLE';
pt(28).ndims     = '2';
pt(28).size      = '[]';

pt(29).blockname = 'Hardware I//O/Output/saturation';
pt(29).paramname = 'UpperLimit';
pt(29).class     = 'scalar';
pt(29).nrows     = 1;
pt(29).ncols     = 1;
pt(29).subsource = 'SS_DOUBLE';
pt(29).ndims     = '2';
pt(29).size      = '[]';

pt(30).blockname = 'Hardware I//O/Output/saturation';
pt(30).paramname = 'LowerLimit';
pt(30).class     = 'scalar';
pt(30).nrows     = 1;
pt(30).ncols     = 1;
pt(30).subsource = 'SS_DOUBLE';
pt(30).ndims     = '2';
pt(30).size      = '[]';

pt(31).blockname = 'Hardware I//O/Output/saturation 2';
pt(31).paramname = 'UpperLimit';
pt(31).class     = 'scalar';
pt(31).nrows     = 1;
pt(31).ncols     = 1;
pt(31).subsource = 'SS_DOUBLE';
pt(31).ndims     = '2';
pt(31).size      = '[]';

pt(32).blockname = 'Hardware I//O/Output/saturation 2';
pt(32).paramname = 'LowerLimit';
pt(32).class     = 'scalar';
pt(32).nrows     = 1;
pt(32).ncols     = 1;
pt(32).subsource = 'SS_DOUBLE';
pt(32).ndims     = '2';
pt(32).size      = '[]';

pt(33).blockname = 'Hardware I//O/Output/saturation1';
pt(33).paramname = 'UpperLimit';
pt(33).class     = 'scalar';
pt(33).nrows     = 1;
pt(33).ncols     = 1;
pt(33).subsource = 'SS_DOUBLE';
pt(33).ndims     = '2';
pt(33).size      = '[]';

pt(34).blockname = 'Hardware I//O/Output/saturation1';
pt(34).paramname = 'LowerLimit';
pt(34).class     = 'scalar';
pt(34).nrows     = 1;
pt(34).ncols     = 1;
pt(34).subsource = 'SS_DOUBLE';
pt(34).ndims     = '2';
pt(34).size      = '[]';

pt(35).blockname = 'Hardware I//O/Output/Ruby-MM ';
pt(35).paramname = 'P1';
pt(35).class     = 'vector';
pt(35).nrows     = 1;
pt(35).ncols     = 7;
pt(35).subsource = 'SS_DOUBLE';
pt(35).ndims     = '2';
pt(35).size      = '[]';

pt(36).blockname = 'Hardware I//O/Output/Ruby-MM ';
pt(36).paramname = 'P2';
pt(36).class     = 'scalar';
pt(36).nrows     = 1;
pt(36).ncols     = 1;
pt(36).subsource = 'SS_DOUBLE';
pt(36).ndims     = '2';
pt(36).size      = '[]';

pt(37).blockname = 'Hardware I//O/Output/Ruby-MM ';
pt(37).paramname = 'P3';
pt(37).class     = 'scalar';
pt(37).nrows     = 1;
pt(37).ncols     = 1;
pt(37).subsource = 'SS_DOUBLE';
pt(37).ndims     = '2';
pt(37).size      = '[]';

pt(38).blockname = 'Hardware I//O/Output/Ruby-MM ';
pt(38).paramname = 'P4';
pt(38).class     = 'vector';
pt(38).nrows     = 1;
pt(38).ncols     = 7;
pt(38).subsource = 'SS_DOUBLE';
pt(38).ndims     = '2';
pt(38).size      = '[]';

pt(39).blockname = 'Hardware I//O/Output/Ruby-MM ';
pt(39).paramname = 'P5';
pt(39).class     = 'vector';
pt(39).nrows     = 1;
pt(39).ncols     = 7;
pt(39).subsource = 'SS_DOUBLE';
pt(39).ndims     = '2';
pt(39).size      = '[]';

pt(40).blockname = 'Hardware I//O/Output/Ruby-MM ';
pt(40).paramname = 'P6';
pt(40).class     = 'scalar';
pt(40).nrows     = 1;
pt(40).ncols     = 1;
pt(40).subsource = 'SS_DOUBLE';
pt(40).ndims     = '2';
pt(40).size      = '[]';

pt(41).blockname = 'Hardware I//O/Output/Ruby-MM ';
pt(41).paramname = 'P7';
pt(41).class     = 'scalar';
pt(41).nrows     = 1;
pt(41).ncols     = 1;
pt(41).subsource = 'SS_DOUBLE';
pt(41).ndims     = '2';
pt(41).size      = '[]';

pt(42).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Enable Observer';
pt(42).paramname = 'Value';
pt(42).class     = 'scalar';
pt(42).nrows     = 1;
pt(42).ncols     = 1;
pt(42).subsource = 'SS_DOUBLE';
pt(42).ndims     = '2';
pt(42).size      = '[]';

pt(43).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Enable Shoreline Gravel';
pt(43).paramname = 'Value';
pt(43).class     = 'scalar';
pt(43).nrows     = 1;
pt(43).ncols     = 1;
pt(43).subsource = 'SS_DOUBLE';
pt(43).ndims     = '2';
pt(43).size      = '[]';

pt(44).blockname = 'Sensor Data and Estimation/Nonlinear Observer/1//Fnf';
pt(44).paramname = 'Gain';
pt(44).class     = 'scalar';
pt(44).nrows     = 1;
pt(44).ncols     = 1;
pt(44).subsource = 'SS_DOUBLE';
pt(44).ndims     = '2';
pt(44).size      = '[]';

pt(45).blockname = 'Sensor Data and Estimation/Nonlinear Observer/rad//s to m//s';
pt(45).paramname = 'Gain';
pt(45).class     = 'vector';
pt(45).nrows     = 1;
pt(45).ncols     = 2;
pt(45).subsource = 'SS_DOUBLE';
pt(45).ndims     = '2';
pt(45).size      = '[]';

pt(46).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(46).paramname = 'P2';
pt(46).class     = 'scalar';
pt(46).nrows     = 1;
pt(46).ncols     = 1;
pt(46).subsource = 'SS_DOUBLE';
pt(46).ndims     = '2';
pt(46).size      = '[]';

pt(47).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(47).paramname = 'P3';
pt(47).class     = 'scalar';
pt(47).nrows     = 1;
pt(47).ncols     = 1;
pt(47).subsource = 'SS_DOUBLE';
pt(47).ndims     = '2';
pt(47).size      = '[]';

pt(48).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(48).paramname = 'P4';
pt(48).class     = 'scalar';
pt(48).nrows     = 1;
pt(48).ncols     = 1;
pt(48).subsource = 'SS_DOUBLE';
pt(48).ndims     = '2';
pt(48).size      = '[]';

pt(49).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(49).paramname = 'P5';
pt(49).class     = 'scalar';
pt(49).nrows     = 1;
pt(49).ncols     = 1;
pt(49).subsource = 'SS_DOUBLE';
pt(49).ndims     = '2';
pt(49).size      = '[]';

pt(50).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(50).paramname = 'P6';
pt(50).class     = 'scalar';
pt(50).nrows     = 1;
pt(50).ncols     = 1;
pt(50).subsource = 'SS_DOUBLE';
pt(50).ndims     = '2';
pt(50).size      = '[]';

pt(51).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(51).paramname = 'P7';
pt(51).class     = 'scalar';
pt(51).nrows     = 1;
pt(51).ncols     = 1;
pt(51).subsource = 'SS_DOUBLE';
pt(51).ndims     = '2';
pt(51).size      = '[]';

pt(52).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(52).paramname = 'P8';
pt(52).class     = 'scalar';
pt(52).nrows     = 1;
pt(52).ncols     = 1;
pt(52).subsource = 'SS_DOUBLE';
pt(52).ndims     = '2';
pt(52).size      = '[]';

pt(53).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(53).paramname = 'P9';
pt(53).class     = 'scalar';
pt(53).nrows     = 1;
pt(53).ncols     = 1;
pt(53).subsource = 'SS_DOUBLE';
pt(53).ndims     = '2';
pt(53).size      = '[]';

pt(54).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(54).paramname = 'P10';
pt(54).class     = 'scalar';
pt(54).nrows     = 1;
pt(54).ncols     = 1;
pt(54).subsource = 'SS_DOUBLE';
pt(54).ndims     = '2';
pt(54).size      = '[]';

pt(55).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(55).paramname = 'P11';
pt(55).class     = 'scalar';
pt(55).nrows     = 1;
pt(55).ncols     = 1;
pt(55).subsource = 'SS_DOUBLE';
pt(55).ndims     = '2';
pt(55).size      = '[]';

pt(56).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(56).paramname = 'P12';
pt(56).class     = 'scalar';
pt(56).nrows     = 1;
pt(56).ncols     = 1;
pt(56).subsource = 'SS_DOUBLE';
pt(56).ndims     = '2';
pt(56).size      = '[]';

pt(57).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(57).paramname = 'P13';
pt(57).class     = 'scalar';
pt(57).nrows     = 1;
pt(57).ncols     = 1;
pt(57).subsource = 'SS_DOUBLE';
pt(57).ndims     = '2';
pt(57).size      = '[]';

pt(58).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(58).paramname = 'P14';
pt(58).class     = 'scalar';
pt(58).nrows     = 1;
pt(58).ncols     = 1;
pt(58).subsource = 'SS_DOUBLE';
pt(58).ndims     = '2';
pt(58).size      = '[]';

pt(59).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(59).paramname = 'P15';
pt(59).class     = 'scalar';
pt(59).nrows     = 1;
pt(59).ncols     = 1;
pt(59).subsource = 'SS_DOUBLE';
pt(59).ndims     = '2';
pt(59).size      = '[]';

pt(60).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(60).paramname = 'P16';
pt(60).class     = 'scalar';
pt(60).nrows     = 1;
pt(60).ncols     = 1;
pt(60).subsource = 'SS_DOUBLE';
pt(60).ndims     = '2';
pt(60).size      = '[]';

pt(61).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(61).paramname = 'P17';
pt(61).class     = 'scalar';
pt(61).nrows     = 1;
pt(61).ncols     = 1;
pt(61).subsource = 'SS_DOUBLE';
pt(61).ndims     = '2';
pt(61).size      = '[]';

pt(62).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(62).paramname = 'P18';
pt(62).class     = 'scalar';
pt(62).nrows     = 1;
pt(62).ncols     = 1;
pt(62).subsource = 'SS_DOUBLE';
pt(62).ndims     = '2';
pt(62).size      = '[]';

pt(63).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(63).paramname = 'P19';
pt(63).class     = 'scalar';
pt(63).nrows     = 1;
pt(63).ncols     = 1;
pt(63).subsource = 'SS_DOUBLE';
pt(63).ndims     = '2';
pt(63).size      = '[]';

pt(64).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(64).paramname = 'P20';
pt(64).class     = 'scalar';
pt(64).nrows     = 1;
pt(64).ncols     = 1;
pt(64).subsource = 'SS_DOUBLE';
pt(64).ndims     = '2';
pt(64).size      = '[]';

pt(65).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(65).paramname = 'P21';
pt(65).class     = 'scalar';
pt(65).nrows     = 1;
pt(65).ncols     = 1;
pt(65).subsource = 'SS_DOUBLE';
pt(65).ndims     = '2';
pt(65).size      = '[]';

pt(66).blockname = 'Sensor Data and Estimation/Nonlinear Observer/If_alpha_observer';
pt(66).paramname = 'P22';
pt(66).class     = 'scalar';
pt(66).nrows     = 1;
pt(66).ncols     = 1;
pt(66).subsource = 'SS_DOUBLE';
pt(66).ndims     = '2';
pt(66).size      = '[]';

pt(67).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Trust Wheelspeeds?';
pt(67).paramname = 'CurrentSetting';
pt(67).class     = 'scalar';
pt(67).nrows     = 1;
pt(67).ncols     = 1;
pt(67).subsource = 'SS_UINT8';
pt(67).ndims     = '2';
pt(67).size      = '[]';

pt(68).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/rad to deg';
pt(68).paramname = 'Gain';
pt(68).class     = 'scalar';
pt(68).nrows     = 1;
pt(68).ncols     = 1;
pt(68).subsource = 'SS_DOUBLE';
pt(68).ndims     = '2';
pt(68).size      = '[]';

pt(69).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter';
pt(69).paramname = 'Numerator';
pt(69).class     = 'scalar';
pt(69).nrows     = 1;
pt(69).ncols     = 1;
pt(69).subsource = 'SS_DOUBLE';
pt(69).ndims     = '2';
pt(69).size      = '[]';

pt(70).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter';
pt(70).paramname = 'Denominator';
pt(70).class     = 'vector';
pt(70).nrows     = 1;
pt(70).ncols     = 2;
pt(70).subsource = 'SS_DOUBLE';
pt(70).ndims     = '2';
pt(70).size      = '[]';

pt(71).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter';
pt(71).paramname = 'InitialStates';
pt(71).class     = 'scalar';
pt(71).nrows     = 1;
pt(71).ncols     = 1;
pt(71).subsource = 'SS_DOUBLE';
pt(71).ndims     = '2';
pt(71).size      = '[]';

pt(72).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter1';
pt(72).paramname = 'Numerator';
pt(72).class     = 'scalar';
pt(72).nrows     = 1;
pt(72).ncols     = 1;
pt(72).subsource = 'SS_DOUBLE';
pt(72).ndims     = '2';
pt(72).size      = '[]';

pt(73).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter1';
pt(73).paramname = 'Denominator';
pt(73).class     = 'vector';
pt(73).nrows     = 1;
pt(73).ncols     = 2;
pt(73).subsource = 'SS_DOUBLE';
pt(73).ndims     = '2';
pt(73).size      = '[]';

pt(74).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter1';
pt(74).paramname = 'InitialStates';
pt(74).class     = 'scalar';
pt(74).nrows     = 1;
pt(74).ncols     = 1;
pt(74).subsource = 'SS_DOUBLE';
pt(74).ndims     = '2';
pt(74).size      = '[]';

pt(75).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter2';
pt(75).paramname = 'Numerator';
pt(75).class     = 'scalar';
pt(75).nrows     = 1;
pt(75).ncols     = 1;
pt(75).subsource = 'SS_DOUBLE';
pt(75).ndims     = '2';
pt(75).size      = '[]';

pt(76).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter2';
pt(76).paramname = 'Denominator';
pt(76).class     = 'vector';
pt(76).nrows     = 1;
pt(76).ncols     = 2;
pt(76).subsource = 'SS_DOUBLE';
pt(76).ndims     = '2';
pt(76).size      = '[]';

pt(77).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter2';
pt(77).paramname = 'InitialStates';
pt(77).class     = 'scalar';
pt(77).nrows     = 1;
pt(77).ncols     = 1;
pt(77).subsource = 'SS_DOUBLE';
pt(77).ndims     = '2';
pt(77).size      = '[]';

pt(78).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter3';
pt(78).paramname = 'Numerator';
pt(78).class     = 'scalar';
pt(78).nrows     = 1;
pt(78).ncols     = 1;
pt(78).subsource = 'SS_DOUBLE';
pt(78).ndims     = '2';
pt(78).size      = '[]';

pt(79).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter3';
pt(79).paramname = 'Denominator';
pt(79).class     = 'vector';
pt(79).nrows     = 1;
pt(79).ncols     = 2;
pt(79).subsource = 'SS_DOUBLE';
pt(79).ndims     = '2';
pt(79).size      = '[]';

pt(80).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter3';
pt(80).paramname = 'InitialStates';
pt(80).class     = 'scalar';
pt(80).nrows     = 1;
pt(80).ncols     = 1;
pt(80).subsource = 'SS_DOUBLE';
pt(80).ndims     = '2';
pt(80).size      = '[]';

pt(81).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter4';
pt(81).paramname = 'Numerator';
pt(81).class     = 'scalar';
pt(81).nrows     = 1;
pt(81).ncols     = 1;
pt(81).subsource = 'SS_DOUBLE';
pt(81).ndims     = '2';
pt(81).size      = '[]';

pt(82).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter4';
pt(82).paramname = 'Denominator';
pt(82).class     = 'vector';
pt(82).nrows     = 1;
pt(82).ncols     = 2;
pt(82).subsource = 'SS_DOUBLE';
pt(82).ndims     = '2';
pt(82).size      = '[]';

pt(83).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Low-pass filter4';
pt(83).paramname = 'InitialStates';
pt(83).class     = 'scalar';
pt(83).nrows     = 1;
pt(83).ncols     = 1;
pt(83).subsource = 'SS_DOUBLE';
pt(83).ndims     = '2';
pt(83).size      = '[]';

pt(84).blockname = 'User Controllers/Cruise Control/Gain';
pt(84).paramname = 'Gain';
pt(84).class     = 'scalar';
pt(84).nrows     = 1;
pt(84).ncols     = 1;
pt(84).subsource = 'SS_DOUBLE';
pt(84).ndims     = '2';
pt(84).size      = '[]';

pt(85).blockname = 'User Controllers/Cruise Control/Quantizer';
pt(85).paramname = 'QuantizationInterval';
pt(85).class     = 'scalar';
pt(85).nrows     = 1;
pt(85).ncols     = 1;
pt(85).subsource = 'SS_DOUBLE';
pt(85).ndims     = '2';
pt(85).size      = '[]';

pt(86).blockname = 'User Controllers/Cruise Control/Switch';
pt(86).paramname = 'Threshold';
pt(86).class     = 'scalar';
pt(86).nrows     = 1;
pt(86).ncols     = 1;
pt(86).subsource = 'SS_DOUBLE';
pt(86).ndims     = '2';
pt(86).size      = '[]';

pt(87).blockname = 'User Controllers/Steering Controller/HAL-9000';
pt(87).paramname = 'enabled';
pt(87).class     = 'scalar';
pt(87).nrows     = 1;
pt(87).ncols     = 1;
pt(87).subsource = 'SS_DOUBLE';
pt(87).ndims     = '2';
pt(87).size      = '[]';

pt(88).blockname = 'User Controllers/Steering Controller/Input Selector';
pt(88).paramname = 'Value';
pt(88).class     = 'scalar';
pt(88).nrows     = 1;
pt(88).ncols     = 1;
pt(88).subsource = 'SS_DOUBLE';
pt(88).ndims     = '2';
pt(88).size      = '[]';

pt(89).blockname = 'User Controllers/Steering Controller/Toe IN Offset';
pt(89).paramname = 'Value';
pt(89).class     = 'scalar';
pt(89).nrows     = 1;
pt(89).ncols     = 1;
pt(89).subsource = 'SS_DOUBLE';
pt(89).ndims     = '2';
pt(89).size      = '[]';

pt(90).blockname = 'User Controllers/Steering Controller/D2R';
pt(90).paramname = 'Gain';
pt(90).class     = 'scalar';
pt(90).nrows     = 1;
pt(90).ncols     = 1;
pt(90).subsource = 'SS_DOUBLE';
pt(90).ndims     = '2';
pt(90).size      = '[]';

pt(91).blockname = 'User Controllers/Steering Controller/Steering Ratio1';
pt(91).paramname = 'Gain';
pt(91).class     = 'scalar';
pt(91).nrows     = 1;
pt(91).ncols     = 1;
pt(91).subsource = 'SS_DOUBLE';
pt(91).ndims     = '2';
pt(91).size      = '[]';

pt(92).blockname = 'User Controllers/Steering Controller/Steering Ratio';
pt(92).paramname = 'Gain';
pt(92).class     = 'scalar';
pt(92).nrows     = 1;
pt(92).ncols     = 1;
pt(92).subsource = 'SS_DOUBLE';
pt(92).ndims     = '2';
pt(92).size      = '[]';

pt(93).blockname = 'User Controllers/Steering Controller/Software Steering Limit';
pt(93).paramname = 'UpperLimit';
pt(93).class     = 'scalar';
pt(93).nrows     = 1;
pt(93).ncols     = 1;
pt(93).subsource = 'SS_DOUBLE';
pt(93).ndims     = '2';
pt(93).size      = '[]';

pt(94).blockname = 'User Controllers/Steering Controller/Software Steering Limit';
pt(94).paramname = 'LowerLimit';
pt(94).class     = 'scalar';
pt(94).nrows     = 1;
pt(94).ncols     = 1;
pt(94).subsource = 'SS_DOUBLE';
pt(94).ndims     = '2';
pt(94).size      = '[]';

pt(95).blockname = 'User Controllers/Steering Controller/Software Steering Limit1';
pt(95).paramname = 'UpperLimit';
pt(95).class     = 'scalar';
pt(95).nrows     = 1;
pt(95).ncols     = 1;
pt(95).subsource = 'SS_DOUBLE';
pt(95).ndims     = '2';
pt(95).size      = '[]';

pt(96).blockname = 'User Controllers/Steering Controller/Software Steering Limit1';
pt(96).paramname = 'LowerLimit';
pt(96).class     = 'scalar';
pt(96).nrows     = 1;
pt(96).ncols     = 1;
pt(96).subsource = 'SS_DOUBLE';
pt(96).ndims     = '2';
pt(96).size      = '[]';

pt(97).blockname = 'User Controllers/Steering Controller/Switch';
pt(97).paramname = 'Threshold';
pt(97).class     = 'scalar';
pt(97).nrows     = 1;
pt(97).ncols     = 1;
pt(97).subsource = 'SS_DOUBLE';
pt(97).ndims     = '2';
pt(97).size      = '[]';

pt(98).blockname = 'User Controllers/Steering Controller/Heavy Filter';
pt(98).paramname = 'Numerator';
pt(98).class     = 'scalar';
pt(98).nrows     = 1;
pt(98).ncols     = 1;
pt(98).subsource = 'SS_DOUBLE';
pt(98).ndims     = '2';
pt(98).size      = '[]';

pt(99).blockname = 'User Controllers/Steering Controller/Heavy Filter';
pt(99).paramname = 'Denominator';
pt(99).class     = 'vector';
pt(99).nrows     = 1;
pt(99).ncols     = 2;
pt(99).subsource = 'SS_DOUBLE';
pt(99).ndims     = '2';
pt(99).size      = '[]';

pt(100).blockname = 'User Controllers/Steering Controller/Heavy Filter';
pt(100).paramname = 'InitialStates';
pt(100).class     = 'scalar';
pt(100).nrows     = 1;
pt(100).ncols     = 1;
pt(100).subsource = 'SS_DOUBLE';
pt(100).ndims     = '2';
pt(100).size      = '[]';

pt(101).blockname = 'User Controllers/Steering Controller/Nominal Filter';
pt(101).paramname = 'Numerator';
pt(101).class     = 'scalar';
pt(101).nrows     = 1;
pt(101).ncols     = 1;
pt(101).subsource = 'SS_DOUBLE';
pt(101).ndims     = '2';
pt(101).size      = '[]';

pt(102).blockname = 'User Controllers/Steering Controller/Nominal Filter';
pt(102).paramname = 'Denominator';
pt(102).class     = 'vector';
pt(102).nrows     = 1;
pt(102).ncols     = 2;
pt(102).subsource = 'SS_DOUBLE';
pt(102).ndims     = '2';
pt(102).size      = '[]';

pt(103).blockname = 'User Controllers/Steering Controller/Nominal Filter';
pt(103).paramname = 'InitialStates';
pt(103).class     = 'scalar';
pt(103).nrows     = 1;
pt(103).ncols     = 1;
pt(103).subsource = 'SS_DOUBLE';
pt(103).ndims     = '2';
pt(103).size      = '[]';

pt(104).blockname = 'Controllers and Output/Steering Controller/LF steering controller/339 current offset';
pt(104).paramname = 'Value';
pt(104).class     = 'scalar';
pt(104).nrows     = 1;
pt(104).ncols     = 1;
pt(104).subsource = 'SS_DOUBLE';
pt(104).ndims     = '2';
pt(104).size      = '[]';

pt(105).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Discrete-Time Integrator';
pt(105).paramname = 'gainval';
pt(105).class     = 'scalar';
pt(105).nrows     = 1;
pt(105).ncols     = 1;
pt(105).subsource = 'SS_DOUBLE';
pt(105).ndims     = '2';
pt(105).size      = '[]';

pt(106).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Discrete-Time Integrator';
pt(106).paramname = 'InitialCondition';
pt(106).class     = 'scalar';
pt(106).nrows     = 1;
pt(106).ncols     = 1;
pt(106).subsource = 'SS_DOUBLE';
pt(106).ndims     = '2';
pt(106).size      = '[]';

pt(107).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Convert to output shaft angle';
pt(107).paramname = 'Gain';
pt(107).class     = 'scalar';
pt(107).nrows     = 1;
pt(107).ncols     = 1;
pt(107).subsource = 'SS_DOUBLE';
pt(107).ndims     = '2';
pt(107).size      = '[]';

pt(108).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Convert to output shaft angle (too)';
pt(108).paramname = 'Gain';
pt(108).class     = 'scalar';
pt(108).nrows     = 1;
pt(108).ncols     = 1;
pt(108).subsource = 'SS_DOUBLE';
pt(108).ndims     = '2';
pt(108).size      = '[]';

pt(109).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Kd';
pt(109).paramname = 'Gain';
pt(109).class     = 'scalar';
pt(109).nrows     = 1;
pt(109).ncols     = 1;
pt(109).subsource = 'SS_DOUBLE';
pt(109).ndims     = '2';
pt(109).size      = '[]';

pt(110).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Ki';
pt(110).paramname = 'Gain';
pt(110).class     = 'scalar';
pt(110).nrows     = 1;
pt(110).ncols     = 1;
pt(110).subsource = 'SS_DOUBLE';
pt(110).ndims     = '2';
pt(110).size      = '[]';

pt(111).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Kp';
pt(111).paramname = 'Gain';
pt(111).class     = 'scalar';
pt(111).nrows     = 1;
pt(111).ncols     = 1;
pt(111).subsource = 'SS_DOUBLE';
pt(111).ndims     = '2';
pt(111).size      = '[]';

pt(112).blockname = 'Controllers and Output/Steering Controller/LF steering controller/accel gain';
pt(112).paramname = 'Gain';
pt(112).class     = 'scalar';
pt(112).nrows     = 1;
pt(112).ncols     = 1;
pt(112).subsource = 'SS_DOUBLE';
pt(112).ndims     = '2';
pt(112).size      = '[]';

pt(113).blockname = 'Controllers and Output/Steering Controller/LF steering controller/current gain';
pt(113).paramname = 'Gain';
pt(113).class     = 'scalar';
pt(113).nrows     = 1;
pt(113).ncols     = 1;
pt(113).subsource = 'SS_DOUBLE';
pt(113).ndims     = '2';
pt(113).size      = '[]';

pt(114).blockname = 'Controllers and Output/Steering Controller/LF steering controller/friction gain';
pt(114).paramname = 'Gain';
pt(114).class     = 'scalar';
pt(114).nrows     = 1;
pt(114).ncols     = 1;
pt(114).subsource = 'SS_DOUBLE';
pt(114).ndims     = '2';
pt(114).size      = '[]';

pt(115).blockname = 'Controllers and Output/Steering Controller/LF steering controller/rate gain';
pt(115).paramname = 'Gain';
pt(115).class     = 'scalar';
pt(115).nrows     = 1;
pt(115).ncols     = 1;
pt(115).subsource = 'SS_DOUBLE';
pt(115).ndims     = '2';
pt(115).size      = '[]';

pt(116).blockname = 'Controllers and Output/Steering Controller/LF steering controller/current limit';
pt(116).paramname = 'UpperLimit';
pt(116).class     = 'scalar';
pt(116).nrows     = 1;
pt(116).ncols     = 1;
pt(116).subsource = 'SS_DOUBLE';
pt(116).ndims     = '2';
pt(116).size      = '[]';

pt(117).blockname = 'Controllers and Output/Steering Controller/LF steering controller/current limit';
pt(117).paramname = 'LowerLimit';
pt(117).class     = 'scalar';
pt(117).nrows     = 1;
pt(117).ncols     = 1;
pt(117).subsource = 'SS_DOUBLE';
pt(117).ndims     = '2';
pt(117).size      = '[]';

pt(118).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter';
pt(118).paramname = 'Numerator';
pt(118).class     = 'vector';
pt(118).nrows     = 1;
pt(118).ncols     = 2;
pt(118).subsource = 'SS_DOUBLE';
pt(118).ndims     = '2';
pt(118).size      = '[]';

pt(119).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter';
pt(119).paramname = 'Denominator';
pt(119).class     = 'vector';
pt(119).nrows     = 1;
pt(119).ncols     = 2;
pt(119).subsource = 'SS_DOUBLE';
pt(119).ndims     = '2';
pt(119).size      = '[]';

pt(120).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter';
pt(120).paramname = 'InitialStates';
pt(120).class     = 'scalar';
pt(120).nrows     = 1;
pt(120).ncols     = 1;
pt(120).subsource = 'SS_DOUBLE';
pt(120).ndims     = '2';
pt(120).size      = '[]';

pt(121).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter 2';
pt(121).paramname = 'Numerator';
pt(121).class     = 'vector';
pt(121).nrows     = 1;
pt(121).ncols     = 2;
pt(121).subsource = 'SS_DOUBLE';
pt(121).ndims     = '2';
pt(121).size      = '[]';

pt(122).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter 2';
pt(122).paramname = 'Denominator';
pt(122).class     = 'vector';
pt(122).nrows     = 1;
pt(122).ncols     = 2;
pt(122).subsource = 'SS_DOUBLE';
pt(122).ndims     = '2';
pt(122).size      = '[]';

pt(123).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter 2';
pt(123).paramname = 'InitialStates';
pt(123).class     = 'scalar';
pt(123).nrows     = 1;
pt(123).ncols     = 1;
pt(123).subsource = 'SS_DOUBLE';
pt(123).ndims     = '2';
pt(123).size      = '[]';

pt(124).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter 3';
pt(124).paramname = 'Numerator';
pt(124).class     = 'vector';
pt(124).nrows     = 1;
pt(124).ncols     = 2;
pt(124).subsource = 'SS_DOUBLE';
pt(124).ndims     = '2';
pt(124).size      = '[]';

pt(125).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter 3';
pt(125).paramname = 'Denominator';
pt(125).class     = 'vector';
pt(125).nrows     = 1;
pt(125).ncols     = 2;
pt(125).subsource = 'SS_DOUBLE';
pt(125).ndims     = '2';
pt(125).size      = '[]';

pt(126).blockname = 'Controllers and Output/Steering Controller/LF steering controller/Derivative & Low-pass filter 3';
pt(126).paramname = 'InitialStates';
pt(126).class     = 'scalar';
pt(126).nrows     = 1;
pt(126).ncols     = 1;
pt(126).subsource = 'SS_DOUBLE';
pt(126).ndims     = '2';
pt(126).size      = '[]';

pt(127).blockname = 'Controllers and Output/Steering Controller/RF steering controller/339 current offset';
pt(127).paramname = 'Value';
pt(127).class     = 'scalar';
pt(127).nrows     = 1;
pt(127).ncols     = 1;
pt(127).subsource = 'SS_DOUBLE';
pt(127).ndims     = '2';
pt(127).size      = '[]';

pt(128).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Discrete-Time Integrator';
pt(128).paramname = 'gainval';
pt(128).class     = 'scalar';
pt(128).nrows     = 1;
pt(128).ncols     = 1;
pt(128).subsource = 'SS_DOUBLE';
pt(128).ndims     = '2';
pt(128).size      = '[]';

pt(129).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Discrete-Time Integrator';
pt(129).paramname = 'InitialCondition';
pt(129).class     = 'scalar';
pt(129).nrows     = 1;
pt(129).ncols     = 1;
pt(129).subsource = 'SS_DOUBLE';
pt(129).ndims     = '2';
pt(129).size      = '[]';

pt(130).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Convert to output shaft angle';
pt(130).paramname = 'Gain';
pt(130).class     = 'scalar';
pt(130).nrows     = 1;
pt(130).ncols     = 1;
pt(130).subsource = 'SS_DOUBLE';
pt(130).ndims     = '2';
pt(130).size      = '[]';

pt(131).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Convert to output shaft angle (too)';
pt(131).paramname = 'Gain';
pt(131).class     = 'scalar';
pt(131).nrows     = 1;
pt(131).ncols     = 1;
pt(131).subsource = 'SS_DOUBLE';
pt(131).ndims     = '2';
pt(131).size      = '[]';

pt(132).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Kd';
pt(132).paramname = 'Gain';
pt(132).class     = 'scalar';
pt(132).nrows     = 1;
pt(132).ncols     = 1;
pt(132).subsource = 'SS_DOUBLE';
pt(132).ndims     = '2';
pt(132).size      = '[]';

pt(133).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Ki';
pt(133).paramname = 'Gain';
pt(133).class     = 'scalar';
pt(133).nrows     = 1;
pt(133).ncols     = 1;
pt(133).subsource = 'SS_DOUBLE';
pt(133).ndims     = '2';
pt(133).size      = '[]';

pt(134).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Kp';
pt(134).paramname = 'Gain';
pt(134).class     = 'scalar';
pt(134).nrows     = 1;
pt(134).ncols     = 1;
pt(134).subsource = 'SS_DOUBLE';
pt(134).ndims     = '2';
pt(134).size      = '[]';

pt(135).blockname = 'Controllers and Output/Steering Controller/RF steering controller/accel gain';
pt(135).paramname = 'Gain';
pt(135).class     = 'scalar';
pt(135).nrows     = 1;
pt(135).ncols     = 1;
pt(135).subsource = 'SS_DOUBLE';
pt(135).ndims     = '2';
pt(135).size      = '[]';

pt(136).blockname = 'Controllers and Output/Steering Controller/RF steering controller/current gain';
pt(136).paramname = 'Gain';
pt(136).class     = 'scalar';
pt(136).nrows     = 1;
pt(136).ncols     = 1;
pt(136).subsource = 'SS_DOUBLE';
pt(136).ndims     = '2';
pt(136).size      = '[]';

pt(137).blockname = 'Controllers and Output/Steering Controller/RF steering controller/friction gain';
pt(137).paramname = 'Gain';
pt(137).class     = 'scalar';
pt(137).nrows     = 1;
pt(137).ncols     = 1;
pt(137).subsource = 'SS_DOUBLE';
pt(137).ndims     = '2';
pt(137).size      = '[]';

pt(138).blockname = 'Controllers and Output/Steering Controller/RF steering controller/rate gain';
pt(138).paramname = 'Gain';
pt(138).class     = 'scalar';
pt(138).nrows     = 1;
pt(138).ncols     = 1;
pt(138).subsource = 'SS_DOUBLE';
pt(138).ndims     = '2';
pt(138).size      = '[]';

pt(139).blockname = 'Controllers and Output/Steering Controller/RF steering controller/current limit';
pt(139).paramname = 'UpperLimit';
pt(139).class     = 'scalar';
pt(139).nrows     = 1;
pt(139).ncols     = 1;
pt(139).subsource = 'SS_DOUBLE';
pt(139).ndims     = '2';
pt(139).size      = '[]';

pt(140).blockname = 'Controllers and Output/Steering Controller/RF steering controller/current limit';
pt(140).paramname = 'LowerLimit';
pt(140).class     = 'scalar';
pt(140).nrows     = 1;
pt(140).ncols     = 1;
pt(140).subsource = 'SS_DOUBLE';
pt(140).ndims     = '2';
pt(140).size      = '[]';

pt(141).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter';
pt(141).paramname = 'Numerator';
pt(141).class     = 'vector';
pt(141).nrows     = 1;
pt(141).ncols     = 2;
pt(141).subsource = 'SS_DOUBLE';
pt(141).ndims     = '2';
pt(141).size      = '[]';

pt(142).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter';
pt(142).paramname = 'Denominator';
pt(142).class     = 'vector';
pt(142).nrows     = 1;
pt(142).ncols     = 2;
pt(142).subsource = 'SS_DOUBLE';
pt(142).ndims     = '2';
pt(142).size      = '[]';

pt(143).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter';
pt(143).paramname = 'InitialStates';
pt(143).class     = 'scalar';
pt(143).nrows     = 1;
pt(143).ncols     = 1;
pt(143).subsource = 'SS_DOUBLE';
pt(143).ndims     = '2';
pt(143).size      = '[]';

pt(144).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter 2';
pt(144).paramname = 'Numerator';
pt(144).class     = 'vector';
pt(144).nrows     = 1;
pt(144).ncols     = 2;
pt(144).subsource = 'SS_DOUBLE';
pt(144).ndims     = '2';
pt(144).size      = '[]';

pt(145).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter 2';
pt(145).paramname = 'Denominator';
pt(145).class     = 'vector';
pt(145).nrows     = 1;
pt(145).ncols     = 2;
pt(145).subsource = 'SS_DOUBLE';
pt(145).ndims     = '2';
pt(145).size      = '[]';

pt(146).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter 2';
pt(146).paramname = 'InitialStates';
pt(146).class     = 'scalar';
pt(146).nrows     = 1;
pt(146).ncols     = 1;
pt(146).subsource = 'SS_DOUBLE';
pt(146).ndims     = '2';
pt(146).size      = '[]';

pt(147).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter 3';
pt(147).paramname = 'Numerator';
pt(147).class     = 'vector';
pt(147).nrows     = 1;
pt(147).ncols     = 2;
pt(147).subsource = 'SS_DOUBLE';
pt(147).ndims     = '2';
pt(147).size      = '[]';

pt(148).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter 3';
pt(148).paramname = 'Denominator';
pt(148).class     = 'vector';
pt(148).nrows     = 1;
pt(148).ncols     = 2;
pt(148).subsource = 'SS_DOUBLE';
pt(148).ndims     = '2';
pt(148).size      = '[]';

pt(149).blockname = 'Controllers and Output/Steering Controller/RF steering controller/Derivative & Low-pass filter 3';
pt(149).paramname = 'InitialStates';
pt(149).class     = 'scalar';
pt(149).nrows     = 1;
pt(149).ncols     = 1;
pt(149).subsource = 'SS_DOUBLE';
pt(149).ndims     = '2';
pt(149).size      = '[]';

pt(150).blockname = 'Controllers and Output/Steering Controller/Steering Geometry & Gearbox Correction/Left Steering Ratio';
pt(150).paramname = 'Gain';
pt(150).class     = 'scalar';
pt(150).nrows     = 1;
pt(150).ncols     = 1;
pt(150).subsource = 'SS_DOUBLE';
pt(150).ndims     = '2';
pt(150).size      = '[]';

pt(151).blockname = 'Controllers and Output/Steering Controller/Steering Geometry & Gearbox Correction/Right Steering Ratio';
pt(151).paramname = 'Gain';
pt(151).class     = 'scalar';
pt(151).nrows     = 1;
pt(151).ncols     = 1;
pt(151).subsource = 'SS_DOUBLE';
pt(151).ndims     = '2';
pt(151).size      = '[]';

pt(152).blockname = 'Controllers and Output/Steering Controller/Steering Geometry & Gearbox Correction/Left Steer Angle Look-up';
pt(152).paramname = 'Table';
pt(152).class     = 'vector';
pt(152).nrows     = 1;
pt(152).ncols     = 101;
pt(152).subsource = 'SS_DOUBLE';
pt(152).ndims     = '2';
pt(152).size      = '[]';

pt(153).blockname = 'Controllers and Output/Steering Controller/Steering Geometry & Gearbox Correction/Left Steer Angle Look-up';
pt(153).paramname = 'BreakpointsForDimension1';
pt(153).class     = 'vector';
pt(153).nrows     = 1;
pt(153).ncols     = 101;
pt(153).subsource = 'SS_DOUBLE';
pt(153).ndims     = '2';
pt(153).size      = '[]';

pt(154).blockname = 'Controllers and Output/Steering Controller/Steering Geometry & Gearbox Correction/Right Steer Angle Look-up';
pt(154).paramname = 'Table';
pt(154).class     = 'vector';
pt(154).nrows     = 1;
pt(154).ncols     = 101;
pt(154).subsource = 'SS_DOUBLE';
pt(154).ndims     = '2';
pt(154).size      = '[]';

pt(155).blockname = 'Controllers and Output/Steering Controller/Steering Geometry & Gearbox Correction/Right Steer Angle Look-up';
pt(155).paramname = 'BreakpointsForDimension1';
pt(155).class     = 'vector';
pt(155).nrows     = 1;
pt(155).ncols     = 101;
pt(155).subsource = 'SS_DOUBLE';
pt(155).ndims     = '2';
pt(155).size      = '[]';

pt(156).blockname = 'Controllers and Output/Steering Controller/standby/Enable Controller';
pt(156).paramname = 'P1';
pt(156).class     = 'scalar';
pt(156).nrows     = 1;
pt(156).ncols     = 1;
pt(156).subsource = 'SS_DOUBLE';
pt(156).ndims     = '2';
pt(156).size      = '[]';

pt(157).blockname = 'Controllers and Output/Steering Controller/standby/Enable Controller';
pt(157).paramname = 'P2';
pt(157).class     = 'scalar';
pt(157).nrows     = 1;
pt(157).ncols     = 1;
pt(157).subsource = 'SS_DOUBLE';
pt(157).ndims     = '2';
pt(157).size      = '[]';

pt(158).blockname = 'Controllers and Output/Steering Controller/standby/Enable Controller';
pt(158).paramname = 'P3';
pt(158).class     = 'scalar';
pt(158).nrows     = 1;
pt(158).ncols     = 1;
pt(158).subsource = 'SS_DOUBLE';
pt(158).ndims     = '2';
pt(158).size      = '[]';

pt(159).blockname = 'Controllers and Output/Steering Controller/standby/Derivative & Low-pass filter';
pt(159).paramname = 'Numerator';
pt(159).class     = 'vector';
pt(159).nrows     = 1;
pt(159).ncols     = 2;
pt(159).subsource = 'SS_DOUBLE';
pt(159).ndims     = '2';
pt(159).size      = '[]';

pt(160).blockname = 'Controllers and Output/Steering Controller/standby/Derivative & Low-pass filter';
pt(160).paramname = 'Denominator';
pt(160).class     = 'vector';
pt(160).nrows     = 1;
pt(160).ncols     = 2;
pt(160).subsource = 'SS_DOUBLE';
pt(160).ndims     = '2';
pt(160).size      = '[]';

pt(161).blockname = 'Controllers and Output/Steering Controller/standby/Derivative & Low-pass filter';
pt(161).paramname = 'InitialStates';
pt(161).class     = 'scalar';
pt(161).nrows     = 1;
pt(161).ncols     = 1;
pt(161).subsource = 'SS_DOUBLE';
pt(161).ndims     = '2';
pt(161).size      = '[]';

pt(162).blockname = 'Controllers and Output/Steering Controller/standby/filter';
pt(162).paramname = 'Numerator';
pt(162).class     = 'scalar';
pt(162).nrows     = 1;
pt(162).ncols     = 1;
pt(162).subsource = 'SS_DOUBLE';
pt(162).ndims     = '2';
pt(162).size      = '[]';

pt(163).blockname = 'Controllers and Output/Steering Controller/standby/filter';
pt(163).paramname = 'Denominator';
pt(163).class     = 'vector';
pt(163).nrows     = 1;
pt(163).ncols     = 2;
pt(163).subsource = 'SS_DOUBLE';
pt(163).ndims     = '2';
pt(163).size      = '[]';

pt(164).blockname = 'Controllers and Output/Steering Controller/standby/filter';
pt(164).paramname = 'InitialStates';
pt(164).class     = 'scalar';
pt(164).nrows     = 1;
pt(164).ncols     = 1;
pt(164).subsource = 'SS_DOUBLE';
pt(164).ndims     = '2';
pt(164).size      = '[]';

pt(165).blockname = 'Controllers and Output/Steering Controller/standby/filter1';
pt(165).paramname = 'Numerator';
pt(165).class     = 'scalar';
pt(165).nrows     = 1;
pt(165).ncols     = 1;
pt(165).subsource = 'SS_DOUBLE';
pt(165).ndims     = '2';
pt(165).size      = '[]';

pt(166).blockname = 'Controllers and Output/Steering Controller/standby/filter1';
pt(166).paramname = 'Denominator';
pt(166).class     = 'vector';
pt(166).nrows     = 1;
pt(166).ncols     = 2;
pt(166).subsource = 'SS_DOUBLE';
pt(166).ndims     = '2';
pt(166).size      = '[]';

pt(167).blockname = 'Controllers and Output/Steering Controller/standby/filter1';
pt(167).paramname = 'InitialStates';
pt(167).class     = 'scalar';
pt(167).nrows     = 1;
pt(167).ncols     = 1;
pt(167).subsource = 'SS_DOUBLE';
pt(167).ndims     = '2';
pt(167).size      = '[]';

pt(168).blockname = 'Controllers and Output/UI Controller/Speedometer/Gain';
pt(168).paramname = 'Gain';
pt(168).class     = 'scalar';
pt(168).nrows     = 1;
pt(168).ncols     = 1;
pt(168).subsource = 'SS_DOUBLE';
pt(168).ndims     = '2';
pt(168).size      = '[]';

pt(169).blockname = 'Hardware I//O/Input/Encoder Board/reset';
pt(169).paramname = 'Value';
pt(169).class     = 'vector';
pt(169).nrows     = 1;
pt(169).ncols     = 5;
pt(169).subsource = 'SS_DOUBLE';
pt(169).ndims     = '2';
pt(169).size      = '[]';

pt(170).blockname = 'Hardware I//O/Input/Encoder Board/encoder board ';
pt(170).paramname = 'P1';
pt(170).class     = 'scalar';
pt(170).nrows     = 1;
pt(170).ncols     = 1;
pt(170).subsource = 'SS_DOUBLE';
pt(170).ndims     = '2';
pt(170).size      = '[]';

pt(171).blockname = 'Hardware I//O/Input/Encoder Board/encoder board ';
pt(171).paramname = 'P2';
pt(171).class     = 'scalar';
pt(171).nrows     = 1;
pt(171).ncols     = 1;
pt(171).subsource = 'SS_DOUBLE';
pt(171).ndims     = '2';
pt(171).size      = '[]';

pt(172).blockname = 'Hardware I//O/Input/Encoder Board/encoder board ';
pt(172).paramname = 'P3';
pt(172).class     = 'scalar';
pt(172).nrows     = 1;
pt(172).ncols     = 1;
pt(172).subsource = 'SS_DOUBLE';
pt(172).ndims     = '2';
pt(172).size      = '[]';

pt(173).blockname = 'Hardware I//O/Input/Encoder Board/encoder board ';
pt(173).paramname = 'P4';
pt(173).class     = 'vector';
pt(173).nrows     = 1;
pt(173).ncols     = 6;
pt(173).subsource = 'SS_DOUBLE';
pt(173).ndims     = '2';
pt(173).size      = '[]';

pt(174).blockname = 'Hardware I//O/Input/Gearbox angle scaling/Pot readings with wheels centered';
pt(174).paramname = 'Value';
pt(174).class     = 'vector';
pt(174).nrows     = 2;
pt(174).ncols     = 1;
pt(174).subsource = 'SS_DOUBLE';
pt(174).ndims     = '2';
pt(174).size      = '[]';

pt(175).blockname = 'Hardware I//O/Input/Gearbox angle scaling/Volts//Rad (Left//right pots wired in reverse)';
pt(175).paramname = 'Gain';
pt(175).class     = 'vector';
pt(175).nrows     = 2;
pt(175).ncols     = 1;
pt(175).subsource = 'SS_DOUBLE';
pt(175).ndims     = '2';
pt(175).size      = '[]';

pt(176).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/handwheel offset';
pt(176).paramname = 'Value';
pt(176).class     = 'scalar';
pt(176).nrows     = 1;
pt(176).ncols     = 1;
pt(176).subsource = 'SS_DOUBLE';
pt(176).ndims     = '2';
pt(176).size      = '[]';

pt(177).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Gear Ratio';
pt(177).paramname = 'Gain';
pt(177).class     = 'scalar';
pt(177).nrows     = 1;
pt(177).ncols     = 1;
pt(177).subsource = 'SS_DOUBLE';
pt(177).ndims     = '2';
pt(177).size      = '[]';

pt(178).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Harmonic Drive Gear Ratio';
pt(178).paramname = 'Gain';
pt(178).class     = 'scalar';
pt(178).nrows     = 1;
pt(178).ncols     = 1;
pt(178).subsource = 'SS_DOUBLE';
pt(178).ndims     = '2';
pt(178).size      = '[]';

pt(179).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Harmonic Drive Gear Ratio1';
pt(179).paramname = 'Gain';
pt(179).class     = 'scalar';
pt(179).nrows     = 1;
pt(179).ncols     = 1;
pt(179).subsource = 'SS_DOUBLE';
pt(179).ndims     = '2';
pt(179).size      = '[]';

pt(180).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/encoder scaling';
pt(180).paramname = 'Gain';
pt(180).class     = 'scalar';
pt(180).nrows     = 1;
pt(180).ncols     = 1;
pt(180).subsource = 'SS_DOUBLE';
pt(180).ndims     = '2';
pt(180).size      = '[]';

pt(181).blockname = 'Hardware I//O/Input/INS scaling/Gain';
pt(181).paramname = 'Gain';
pt(181).class     = 'scalar';
pt(181).nrows     = 1;
pt(181).ncols     = 1;
pt(181).subsource = 'SS_DOUBLE';
pt(181).ndims     = '2';
pt(181).size      = '[]';

pt(182).blockname = 'Hardware I//O/Input/INS scaling/Gain1';
pt(182).paramname = 'Gain';
pt(182).class     = 'scalar';
pt(182).nrows     = 1;
pt(182).ncols     = 1;
pt(182).subsource = 'SS_DOUBLE';
pt(182).ndims     = '2';
pt(182).size      = '[]';

pt(183).blockname = 'Hardware I//O/Input/INS scaling/Gain2';
pt(183).paramname = 'Gain';
pt(183).class     = 'scalar';
pt(183).nrows     = 1;
pt(183).ncols     = 1;
pt(183).subsource = 'SS_DOUBLE';
pt(183).ndims     = '2';
pt(183).size      = '[]';

pt(184).blockname = 'Hardware I//O/Input/INS scaling/Gain3';
pt(184).paramname = 'Gain';
pt(184).class     = 'scalar';
pt(184).nrows     = 1;
pt(184).ncols     = 1;
pt(184).subsource = 'SS_DOUBLE';
pt(184).ndims     = '2';
pt(184).size      = '[]';

pt(185).blockname = 'Hardware I//O/Input/INS scaling/Gain4';
pt(185).paramname = 'Gain';
pt(185).class     = 'scalar';
pt(185).nrows     = 1;
pt(185).ncols     = 1;
pt(185).subsource = 'SS_DOUBLE';
pt(185).ndims     = '2';
pt(185).size      = '[]';

pt(186).blockname = 'Hardware I//O/Input/INS scaling/Gain5';
pt(186).paramname = 'Gain';
pt(186).class     = 'scalar';
pt(186).nrows     = 1;
pt(186).ncols     = 1;
pt(186).subsource = 'SS_DOUBLE';
pt(186).ndims     = '2';
pt(186).size      = '[]';

pt(187).blockname = 'Hardware I//O/Input/INS scaling/deg to rad';
pt(187).paramname = 'Gain';
pt(187).class     = 'scalar';
pt(187).nrows     = 1;
pt(187).ncols     = 1;
pt(187).subsource = 'SS_DOUBLE';
pt(187).ndims     = '2';
pt(187).size      = '[]';

pt(188).blockname = 'Hardware I//O/Input/Rear Wheel Speed Processing/de-quadrature';
pt(188).paramname = 'Gain';
pt(188).class     = 'scalar';
pt(188).nrows     = 1;
pt(188).ncols     = 1;
pt(188).subsource = 'SS_DOUBLE';
pt(188).ndims     = '2';
pt(188).size      = '[]';

pt(189).blockname = 'Hardware I//O/Input/Steering Motor Encoder Initialization/gearbox ratio';
pt(189).paramname = 'Gain';
pt(189).class     = 'scalar';
pt(189).nrows     = 1;
pt(189).ncols     = 1;
pt(189).subsource = 'SS_DOUBLE';
pt(189).ndims     = '2';
pt(189).size      = '[]';

pt(190).blockname = 'Hardware I//O/Input/Steering Motor Encoder Initialization/gearbox ratio1';
pt(190).paramname = 'Gain';
pt(190).class     = 'scalar';
pt(190).nrows     = 1;
pt(190).ncols     = 1;
pt(190).subsource = 'SS_DOUBLE';
pt(190).ndims     = '2';
pt(190).size      = '[]';

pt(191).blockname = 'Hardware I//O/Input/Subsystem1/Constant';
pt(191).paramname = 'Value';
pt(191).class     = 'scalar';
pt(191).nrows     = 1;
pt(191).ncols     = 1;
pt(191).subsource = 'SS_DOUBLE';
pt(191).ndims     = '2';
pt(191).size      = '[]';

pt(192).blockname = 'Hardware I//O/Input/Subsystem2/Constant';
pt(192).paramname = 'Value';
pt(192).class     = 'scalar';
pt(192).nrows     = 1;
pt(192).ncols     = 1;
pt(192).subsource = 'SS_DOUBLE';
pt(192).ndims     = '2';
pt(192).size      = '[]';

pt(193).blockname = 'Hardware I//O/Input/VS330 GPS Data/Roll Axis Reversed';
pt(193).paramname = 'Gain';
pt(193).class     = 'scalar';
pt(193).nrows     = 1;
pt(193).ncols     = 1;
pt(193).subsource = 'SS_DOUBLE';
pt(193).ndims     = '2';
pt(193).size      = '[]';

pt(194).blockname = 'Hardware I//O/Input/VSBC Analog Demux/Constant';
pt(194).paramname = 'Value';
pt(194).class     = 'scalar';
pt(194).nrows     = 1;
pt(194).ncols     = 1;
pt(194).subsource = 'SS_DOUBLE';
pt(194).ndims     = '2';
pt(194).size      = '[]';

pt(195).blockname = 'Hardware I//O/Input/VSBC Analog Demux/Voltage to Rad';
pt(195).paramname = 'Gain';
pt(195).class     = 'scalar';
pt(195).nrows     = 1;
pt(195).ncols     = 1;
pt(195).subsource = 'SS_DOUBLE';
pt(195).ndims     = '2';
pt(195).size      = '[]';

pt(196).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(196).paramname = 'P1';
pt(196).class     = 'vector';
pt(196).nrows     = 1;
pt(196).ncols     = 4;
pt(196).subsource = 'SS_DOUBLE';
pt(196).ndims     = '2';
pt(196).size      = '[]';

pt(197).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(197).paramname = 'P2';
pt(197).class     = 'vector';
pt(197).nrows     = 1;
pt(197).ncols     = 4;
pt(197).subsource = 'SS_DOUBLE';
pt(197).ndims     = '2';
pt(197).size      = '[]';

pt(198).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(198).paramname = 'P4';
pt(198).class     = 'vector';
pt(198).nrows     = 1;
pt(198).ncols     = 6;
pt(198).subsource = 'SS_DOUBLE';
pt(198).ndims     = '2';
pt(198).size      = '[]';

pt(199).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(199).paramname = 'P11';
pt(199).class     = 'vector';
pt(199).nrows     = 1;
pt(199).ncols     = 6;
pt(199).subsource = 'SS_DOUBLE';
pt(199).ndims     = '2';
pt(199).size      = '[]';

pt(200).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(200).paramname = 'P15';
pt(200).class     = 'scalar';
pt(200).nrows     = 1;
pt(200).ncols     = 1;
pt(200).subsource = 'SS_DOUBLE';
pt(200).ndims     = '2';
pt(200).size      = '[]';

pt(201).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(201).paramname = 'P16';
pt(201).class     = 'scalar';
pt(201).nrows     = 1;
pt(201).ncols     = 1;
pt(201).subsource = 'SS_DOUBLE';
pt(201).ndims     = '2';
pt(201).size      = '[]';

pt(202).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(202).paramname = 'P17';
pt(202).class     = 'scalar';
pt(202).nrows     = 1;
pt(202).ncols     = 1;
pt(202).subsource = 'SS_DOUBLE';
pt(202).ndims     = '2';
pt(202).size      = '[]';

pt(203).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(203).paramname = 'P18';
pt(203).class     = 'scalar';
pt(203).nrows     = 1;
pt(203).ncols     = 1;
pt(203).subsource = 'SS_DOUBLE';
pt(203).ndims     = '2';
pt(203).size      = '[]';

pt(204).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(204).paramname = 'P19';
pt(204).class     = 'scalar';
pt(204).nrows     = 1;
pt(204).ncols     = 1;
pt(204).subsource = 'SS_DOUBLE';
pt(204).ndims     = '2';
pt(204).size      = '[]';

pt(205).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(205).paramname = 'P20';
pt(205).class     = 'scalar';
pt(205).nrows     = 1;
pt(205).ncols     = 1;
pt(205).subsource = 'SS_DOUBLE';
pt(205).ndims     = '2';
pt(205).size      = '[]';

pt(206).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(206).paramname = 'P21';
pt(206).class     = 'scalar';
pt(206).nrows     = 1;
pt(206).ncols     = 1;
pt(206).subsource = 'SS_DOUBLE';
pt(206).ndims     = '2';
pt(206).size      = '[]';

pt(207).blockname = 'Hardware I//O/Input/Wheel Force Transducers/Setup ';
pt(207).paramname = 'P22';
pt(207).class     = 'scalar';
pt(207).nrows     = 1;
pt(207).ncols     = 1;
pt(207).subsource = 'SS_DOUBLE';
pt(207).ndims     = '2';
pt(207).size      = '[]';

pt(208).blockname = 'Hardware I//O/Input/current scaling/Constant';
pt(208).paramname = 'Value';
pt(208).class     = 'scalar';
pt(208).nrows     = 1;
pt(208).ncols     = 1;
pt(208).subsource = 'SS_DOUBLE';
pt(208).ndims     = '2';
pt(208).size      = '[]';

pt(209).blockname = 'Hardware I//O/Input/current scaling/current scaling';
pt(209).paramname = 'Gain';
pt(209).class     = 'scalar';
pt(209).nrows     = 1;
pt(209).ncols     = 1;
pt(209).subsource = 'SS_DOUBLE';
pt(209).ndims     = '2';
pt(209).size      = '[]';

pt(210).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/CG height';
pt(210).paramname = 'Value';
pt(210).class     = 'scalar';
pt(210).nrows     = 1;
pt(210).ncols     = 1;
pt(210).subsource = 'SS_DOUBLE';
pt(210).ndims     = '2';
pt(210).size      = '[]';

pt(211).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Fnf_nom';
pt(211).paramname = 'Value';
pt(211).class     = 'scalar';
pt(211).nrows     = 1;
pt(211).ncols     = 1;
pt(211).subsource = 'SS_DOUBLE';
pt(211).ndims     = '2';
pt(211).size      = '[]';

pt(212).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Fnr_nom';
pt(212).paramname = 'Value';
pt(212).class     = 'scalar';
pt(212).nrows     = 1;
pt(212).ncols     = 1;
pt(212).subsource = 'SS_DOUBLE';
pt(212).ndims     = '2';
pt(212).size      = '[]';

pt(213).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Front Mass';
pt(213).paramname = 'Value';
pt(213).class     = 'scalar';
pt(213).nrows     = 1;
pt(213).ncols     = 1;
pt(213).subsource = 'SS_DOUBLE';
pt(213).ndims     = '2';
pt(213).size      = '[]';

pt(214).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Roll Stiffness';
pt(214).paramname = 'Value';
pt(214).class     = 'scalar';
pt(214).nrows     = 1;
pt(214).ncols     = 1;
pt(214).subsource = 'SS_DOUBLE';
pt(214).ndims     = '2';
pt(214).size      = '[]';

pt(215).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Track Width';
pt(215).paramname = 'Value';
pt(215).class     = 'scalar';
pt(215).nrows     = 1;
pt(215).ncols     = 1;
pt(215).subsource = 'SS_DOUBLE';
pt(215).ndims     = '2';
pt(215).size      = '[]';

pt(216).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Track Width1';
pt(216).paramname = 'Value';
pt(216).class     = 'scalar';
pt(216).nrows     = 1;
pt(216).ncols     = 1;
pt(216).subsource = 'SS_DOUBLE';
pt(216).ndims     = '2';
pt(216).size      = '[]';

pt(217).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Saturation';
pt(217).paramname = 'UpperLimit';
pt(217).class     = 'scalar';
pt(217).nrows     = 1;
pt(217).ncols     = 1;
pt(217).subsource = 'SS_DOUBLE';
pt(217).ndims     = '2';
pt(217).size      = '[]';

pt(218).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Normal Loads/Saturation';
pt(218).paramname = 'LowerLimit';
pt(218).class     = 'scalar';
pt(218).nrows     = 1;
pt(218).ncols     = 1;
pt(218).subsource = 'SS_DOUBLE';
pt(218).ndims     = '2';
pt(218).size      = '[]';

pt(219).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations';
pt(219).paramname = 'ICPrevScaledInput';
pt(219).class     = 'scalar';
pt(219).nrows     = 1;
pt(219).ncols     = 1;
pt(219).subsource = 'SS_DOUBLE';
pt(219).ndims     = '2';
pt(219).size      = '[]';

pt(220).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations1';
pt(220).paramname = 'ICPrevScaledInput';
pt(220).class     = 'scalar';
pt(220).nrows     = 1;
pt(220).ncols     = 1;
pt(220).subsource = 'SS_DOUBLE';
pt(220).ndims     = '2';
pt(220).size      = '[]';

pt(221).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities';
pt(221).paramname = 'ICPrevScaledInput';
pt(221).class     = 'scalar';
pt(221).nrows     = 1;
pt(221).ncols     = 1;
pt(221).subsource = 'SS_DOUBLE';
pt(221).ndims     = '2';
pt(221).size      = '[]';

pt(222).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities1';
pt(222).paramname = 'ICPrevScaledInput';
pt(222).class     = 'scalar';
pt(222).nrows     = 1;
pt(222).ncols     = 1;
pt(222).subsource = 'SS_DOUBLE';
pt(222).ndims     = '2';
pt(222).size      = '[]';

pt(223).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Dead Zone';
pt(223).paramname = 'LowerValue';
pt(223).class     = 'scalar';
pt(223).nrows     = 1;
pt(223).ncols     = 1;
pt(223).subsource = 'SS_DOUBLE';
pt(223).ndims     = '2';
pt(223).size      = '[]';

pt(224).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Dead Zone';
pt(224).paramname = 'UpperValue';
pt(224).class     = 'scalar';
pt(224).nrows     = 1;
pt(224).ncols     = 1;
pt(224).subsource = 'SS_DOUBLE';
pt(224).ndims     = '2';
pt(224).size      = '[]';

pt(225).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Dead Zone1';
pt(225).paramname = 'LowerValue';
pt(225).class     = 'scalar';
pt(225).nrows     = 1;
pt(225).ncols     = 1;
pt(225).subsource = 'SS_DOUBLE';
pt(225).ndims     = '2';
pt(225).size      = '[]';

pt(226).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Dead Zone1';
pt(226).paramname = 'UpperValue';
pt(226).class     = 'scalar';
pt(226).nrows     = 1;
pt(226).ncols     = 1;
pt(226).subsource = 'SS_DOUBLE';
pt(226).ndims     = '2';
pt(226).size      = '[]';

pt(227).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable';
pt(227).paramname = 'Gain';
pt(227).class     = 'scalar';
pt(227).nrows     = 1;
pt(227).ncols     = 1;
pt(227).subsource = 'SS_DOUBLE';
pt(227).ndims     = '2';
pt(227).size      = '[]';

pt(228).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable1';
pt(228).paramname = 'Gain';
pt(228).class     = 'scalar';
pt(228).nrows     = 1;
pt(228).ncols     = 1;
pt(228).subsource = 'SS_DOUBLE';
pt(228).ndims     = '2';
pt(228).size      = '[]';

pt(229).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable2';
pt(229).paramname = 'Gain';
pt(229).class     = 'scalar';
pt(229).nrows     = 1;
pt(229).ncols     = 1;
pt(229).subsource = 'SS_DOUBLE';
pt(229).ndims     = '2';
pt(229).size      = '[]';

pt(230).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable3';
pt(230).paramname = 'Gain';
pt(230).class     = 'scalar';
pt(230).nrows     = 1;
pt(230).ncols     = 1;
pt(230).subsource = 'SS_DOUBLE';
pt(230).ndims     = '2';
pt(230).size      = '[]';

pt(231).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable4';
pt(231).paramname = 'Gain';
pt(231).class     = 'scalar';
pt(231).nrows     = 1;
pt(231).ncols     = 1;
pt(231).subsource = 'SS_DOUBLE';
pt(231).ndims     = '2';
pt(231).size      = '[]';

pt(232).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable5';
pt(232).paramname = 'Gain';
pt(232).class     = 'scalar';
pt(232).nrows     = 1;
pt(232).ncols     = 1;
pt(232).subsource = 'SS_DOUBLE';
pt(232).ndims     = '2';
pt(232).size      = '[]';

pt(233).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable6';
pt(233).paramname = 'Gain';
pt(233).class     = 'scalar';
pt(233).nrows     = 1;
pt(233).ncols     = 1;
pt(233).subsource = 'SS_DOUBLE';
pt(233).ndims     = '2';
pt(233).size      = '[]';

pt(234).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Disable7';
pt(234).paramname = 'Gain';
pt(234).class     = 'scalar';
pt(234).nrows     = 1;
pt(234).ncols     = 1;
pt(234).subsource = 'SS_DOUBLE';
pt(234).ndims     = '2';
pt(234).size      = '[]';

pt(235).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Damping';
pt(235).paramname = 'Gain';
pt(235).class     = 'scalar';
pt(235).nrows     = 1;
pt(235).ncols     = 1;
pt(235).subsource = 'SS_DOUBLE';
pt(235).ndims     = '2';
pt(235).size      = '[]';

pt(236).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Damping1';
pt(236).paramname = 'Gain';
pt(236).class     = 'scalar';
pt(236).nrows     = 1;
pt(236).ncols     = 1;
pt(236).subsource = 'SS_DOUBLE';
pt(236).ndims     = '2';
pt(236).size      = '[]';

pt(237).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Friction';
pt(237).paramname = 'Gain';
pt(237).class     = 'scalar';
pt(237).nrows     = 1;
pt(237).ncols     = 1;
pt(237).subsource = 'SS_DOUBLE';
pt(237).ndims     = '2';
pt(237).size      = '[]';

pt(238).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Friction1';
pt(238).paramname = 'Gain';
pt(238).class     = 'scalar';
pt(238).nrows     = 1;
pt(238).ncols     = 1;
pt(238).subsource = 'SS_DOUBLE';
pt(238).ndims     = '2';
pt(238).size      = '[]';

pt(239).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Inertia';
pt(239).paramname = 'Gain';
pt(239).class     = 'scalar';
pt(239).nrows     = 1;
pt(239).ncols     = 1;
pt(239).subsource = 'SS_DOUBLE';
pt(239).ndims     = '2';
pt(239).size      = '[]';

pt(240).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Inertia1';
pt(240).paramname = 'Gain';
pt(240).class     = 'scalar';
pt(240).nrows     = 1;
pt(240).ncols     = 1;
pt(240).subsource = 'SS_DOUBLE';
pt(240).ndims     = '2';
pt(240).size      = '[]';

pt(241).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter';
pt(241).paramname = 'Numerator';
pt(241).class     = 'scalar';
pt(241).nrows     = 1;
pt(241).ncols     = 1;
pt(241).subsource = 'SS_DOUBLE';
pt(241).ndims     = '2';
pt(241).size      = '[]';

pt(242).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter';
pt(242).paramname = 'Denominator';
pt(242).class     = 'vector';
pt(242).nrows     = 1;
pt(242).ncols     = 2;
pt(242).subsource = 'SS_DOUBLE';
pt(242).ndims     = '2';
pt(242).size      = '[]';

pt(243).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter';
pt(243).paramname = 'InitialStates';
pt(243).class     = 'scalar';
pt(243).nrows     = 1;
pt(243).ncols     = 1;
pt(243).subsource = 'SS_DOUBLE';
pt(243).ndims     = '2';
pt(243).size      = '[]';

pt(244).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter1';
pt(244).paramname = 'Numerator';
pt(244).class     = 'scalar';
pt(244).nrows     = 1;
pt(244).ncols     = 1;
pt(244).subsource = 'SS_DOUBLE';
pt(244).ndims     = '2';
pt(244).size      = '[]';

pt(245).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter1';
pt(245).paramname = 'Denominator';
pt(245).class     = 'vector';
pt(245).nrows     = 1;
pt(245).ncols     = 2;
pt(245).subsource = 'SS_DOUBLE';
pt(245).ndims     = '2';
pt(245).size      = '[]';

pt(246).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter1';
pt(246).paramname = 'InitialStates';
pt(246).class     = 'scalar';
pt(246).nrows     = 1;
pt(246).ncols     = 1;
pt(246).subsource = 'SS_DOUBLE';
pt(246).ndims     = '2';
pt(246).size      = '[]';

pt(247).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter2';
pt(247).paramname = 'Numerator';
pt(247).class     = 'scalar';
pt(247).nrows     = 1;
pt(247).ncols     = 1;
pt(247).subsource = 'SS_DOUBLE';
pt(247).ndims     = '2';
pt(247).size      = '[]';

pt(248).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter2';
pt(248).paramname = 'Denominator';
pt(248).class     = 'vector';
pt(248).nrows     = 1;
pt(248).ncols     = 2;
pt(248).subsource = 'SS_DOUBLE';
pt(248).ndims     = '2';
pt(248).size      = '[]';

pt(249).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter2';
pt(249).paramname = 'InitialStates';
pt(249).class     = 'scalar';
pt(249).nrows     = 1;
pt(249).ncols     = 1;
pt(249).subsource = 'SS_DOUBLE';
pt(249).ndims     = '2';
pt(249).size      = '[]';

pt(250).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter3';
pt(250).paramname = 'Numerator';
pt(250).class     = 'scalar';
pt(250).nrows     = 1;
pt(250).ncols     = 1;
pt(250).subsource = 'SS_DOUBLE';
pt(250).ndims     = '2';
pt(250).size      = '[]';

pt(251).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter3';
pt(251).paramname = 'Denominator';
pt(251).class     = 'vector';
pt(251).nrows     = 1;
pt(251).ncols     = 2;
pt(251).subsource = 'SS_DOUBLE';
pt(251).ndims     = '2';
pt(251).size      = '[]';

pt(252).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Low-pass filter3';
pt(252).paramname = 'InitialStates';
pt(252).class     = 'scalar';
pt(252).nrows     = 1;
pt(252).ncols     = 1;
pt(252).subsource = 'SS_DOUBLE';
pt(252).ndims     = '2';
pt(252).size      = '[]';

pt(253).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Left Steering Ratio';
pt(253).paramname = 'Gain';
pt(253).class     = 'scalar';
pt(253).nrows     = 1;
pt(253).ncols     = 1;
pt(253).subsource = 'SS_DOUBLE';
pt(253).ndims     = '2';
pt(253).size      = '[]';

pt(254).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Right Steering Ratio';
pt(254).paramname = 'Gain';
pt(254).class     = 'scalar';
pt(254).nrows     = 1;
pt(254).ncols     = 1;
pt(254).subsource = 'SS_DOUBLE';
pt(254).ndims     = '2';
pt(254).size      = '[]';

pt(255).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Left Arm Length Look-up';
pt(255).paramname = 'Table';
pt(255).class     = 'vector';
pt(255).nrows     = 1;
pt(255).ncols     = 101;
pt(255).subsource = 'SS_DOUBLE';
pt(255).ndims     = '2';
pt(255).size      = '[]';

pt(256).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Left Arm Length Look-up';
pt(256).paramname = 'BreakpointsForDimension1';
pt(256).class     = 'vector';
pt(256).nrows     = 1;
pt(256).ncols     = 101;
pt(256).subsource = 'SS_DOUBLE';
pt(256).ndims     = '2';
pt(256).size      = '[]';

pt(257).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Right Arm Length Look-up';
pt(257).paramname = 'Table';
pt(257).class     = 'vector';
pt(257).nrows     = 1;
pt(257).ncols     = 101;
pt(257).subsource = 'SS_DOUBLE';
pt(257).ndims     = '2';
pt(257).size      = '[]';

pt(258).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steer Torque Calculation/Right Arm Length Look-up';
pt(258).paramname = 'BreakpointsForDimension1';
pt(258).class     = 'vector';
pt(258).nrows     = 1;
pt(258).ncols     = 101;
pt(258).subsource = 'SS_DOUBLE';
pt(258).ndims     = '2';
pt(258).size      = '[]';

pt(259).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Left Steering Ratio';
pt(259).paramname = 'Gain';
pt(259).class     = 'scalar';
pt(259).nrows     = 1;
pt(259).ncols     = 1;
pt(259).subsource = 'SS_DOUBLE';
pt(259).ndims     = '2';
pt(259).size      = '[]';

pt(260).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/R2D';
pt(260).paramname = 'Gain';
pt(260).class     = 'scalar';
pt(260).nrows     = 1;
pt(260).ncols     = 1;
pt(260).subsource = 'SS_DOUBLE';
pt(260).ndims     = '2';
pt(260).size      = '[]';

pt(261).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Right Steering Ratio';
pt(261).paramname = 'Gain';
pt(261).class     = 'scalar';
pt(261).nrows     = 1;
pt(261).ncols     = 1;
pt(261).subsource = 'SS_DOUBLE';
pt(261).ndims     = '2';
pt(261).size      = '[]';

pt(262).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Left Steer Angle Look-up';
pt(262).paramname = 'Table';
pt(262).class     = 'vector';
pt(262).nrows     = 1;
pt(262).ncols     = 101;
pt(262).subsource = 'SS_DOUBLE';
pt(262).ndims     = '2';
pt(262).size      = '[]';

pt(263).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Left Steer Angle Look-up';
pt(263).paramname = 'BreakpointsForDimension1';
pt(263).class     = 'vector';
pt(263).nrows     = 1;
pt(263).ncols     = 101;
pt(263).subsource = 'SS_DOUBLE';
pt(263).ndims     = '2';
pt(263).size      = '[]';

pt(264).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Right Steer Angle Look-up';
pt(264).paramname = 'Table';
pt(264).class     = 'vector';
pt(264).nrows     = 1;
pt(264).ncols     = 101;
pt(264).subsource = 'SS_DOUBLE';
pt(264).ndims     = '2';
pt(264).size      = '[]';

pt(265).blockname = 'Sensor Data and Estimation/Steering Kinematics/Steering Angle Calculation/Right Steer Angle Look-up';
pt(265).paramname = 'BreakpointsForDimension1';
pt(265).class     = 'vector';
pt(265).nrows     = 1;
pt(265).ncols     = 101;
pt(265).subsource = 'SS_DOUBLE';
pt(265).ndims     = '2';
pt(265).size      = '[]';

pt(266).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/Select: Yaw Rate/Gain';
pt(266).paramname = 'Gain';
pt(266).class     = 'vector';
pt(266).nrows     = 1;
pt(266).ncols     = 6;
pt(266).subsource = 'SS_DOUBLE';
pt(266).ndims     = '2';
pt(266).size      = '[]';

pt(267).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Roll Gyro Sf';
pt(267).paramname = 'Value';
pt(267).class     = 'scalar';
pt(267).nrows     = 1;
pt(267).ncols     = 1;
pt(267).subsource = 'SS_DOUBLE';
pt(267).ndims     = '2';
pt(267).size      = '[]';

pt(268).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad1';
pt(268).paramname = 'Gain';
pt(268).class     = 'scalar';
pt(268).nrows     = 1;
pt(268).ncols     = 1;
pt(268).subsource = 'SS_DOUBLE';
pt(268).ndims     = '2';
pt(268).size      = '[]';

pt(269).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad2';
pt(269).paramname = 'Gain';
pt(269).class     = 'scalar';
pt(269).nrows     = 1;
pt(269).ncols     = 1;
pt(269).subsource = 'SS_DOUBLE';
pt(269).ndims     = '2';
pt(269).size      = '[]';

pt(270).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad3';
pt(270).paramname = 'Gain';
pt(270).class     = 'scalar';
pt(270).nrows     = 1;
pt(270).ncols     = 1;
pt(270).subsource = 'SS_DOUBLE';
pt(270).ndims     = '2';
pt(270).size      = '[]';

pt(271).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad4';
pt(271).paramname = 'Gain';
pt(271).class     = 'scalar';
pt(271).nrows     = 1;
pt(271).ncols     = 1;
pt(271).subsource = 'SS_DOUBLE';
pt(271).ndims     = '2';
pt(271).size      = '[]';

pt(272).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad5';
pt(272).paramname = 'Gain';
pt(272).class     = 'scalar';
pt(272).nrows     = 1;
pt(272).ncols     = 1;
pt(272).subsource = 'SS_DOUBLE';
pt(272).ndims     = '2';
pt(272).size      = '[]';

pt(273).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad6';
pt(273).paramname = 'Gain';
pt(273).class     = 'scalar';
pt(273).nrows     = 1;
pt(273).ncols     = 1;
pt(273).subsource = 'SS_DOUBLE';
pt(273).ndims     = '2';
pt(273).size      = '[]';

pt(274).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/DegtoRad7';
pt(274).paramname = 'Gain';
pt(274).class     = 'scalar';
pt(274).nrows     = 1;
pt(274).ncols     = 1;
pt(274).subsource = 'SS_DOUBLE';
pt(274).ndims     = '2';
pt(274).size      = '[]';

pt(275).blockname = 'User Controllers/Cruise Control/Pedal Latch/Out1';
pt(275).paramname = 'InitialOutput';
pt(275).class     = 'scalar';
pt(275).nrows     = 1;
pt(275).ncols     = 1;
pt(275).subsource = 'SS_DOUBLE';
pt(275).ndims     = '2';
pt(275).size      = '[]';

pt(276).blockname = 'User Controllers/Cruise Control/Speed Latch/Out1';
pt(276).paramname = 'InitialOutput';
pt(276).class     = 'scalar';
pt(276).nrows     = 1;
pt(276).ncols     = 1;
pt(276).subsource = 'SS_DOUBLE';
pt(276).ndims     = '2';
pt(276).size      = '[]';

pt(277).blockname = 'User Controllers/Steering Controller/HAL-9000/Chirp Signal';
pt(277).paramname = 'f1';
pt(277).class     = 'scalar';
pt(277).nrows     = 1;
pt(277).ncols     = 1;
pt(277).subsource = 'SS_DOUBLE';
pt(277).ndims     = '2';
pt(277).size      = '[]';

pt(278).blockname = 'User Controllers/Steering Controller/HAL-9000/Chirp Signal';
pt(278).paramname = 'T';
pt(278).class     = 'scalar';
pt(278).nrows     = 1;
pt(278).ncols     = 1;
pt(278).subsource = 'SS_DOUBLE';
pt(278).ndims     = '2';
pt(278).size      = '[]';

pt(279).blockname = 'User Controllers/Steering Controller/HAL-9000/Chirp Signal';
pt(279).paramname = 'f2';
pt(279).class     = 'scalar';
pt(279).nrows     = 1;
pt(279).ncols     = 1;
pt(279).subsource = 'SS_DOUBLE';
pt(279).ndims     = '2';
pt(279).size      = '[]';

pt(280).blockname = 'User Controllers/Steering Controller/HAL-9000/Amplitude';
pt(280).paramname = 'Value';
pt(280).class     = 'scalar';
pt(280).nrows     = 1;
pt(280).ncols     = 1;
pt(280).subsource = 'SS_DOUBLE';
pt(280).ndims     = '2';
pt(280).size      = '[]';

pt(281).blockname = 'User Controllers/Steering Controller/HAL-9000/Delay';
pt(281).paramname = 'Value';
pt(281).class     = 'scalar';
pt(281).nrows     = 1;
pt(281).ncols     = 1;
pt(281).subsource = 'SS_DOUBLE';
pt(281).ndims     = '2';
pt(281).size      = '[]';

pt(282).blockname = 'User Controllers/Steering Controller/HAL-9000/Maneuver';
pt(282).paramname = 'Value';
pt(282).class     = 'scalar';
pt(282).nrows     = 1;
pt(282).ncols     = 1;
pt(282).subsource = 'SS_DOUBLE';
pt(282).ndims     = '2';
pt(282).size      = '[]';

pt(283).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Encoder Initialization/Encoder Initializer';
pt(283).paramname = 'P2';
pt(283).class     = 'scalar';
pt(283).nrows     = 1;
pt(283).ncols     = 1;
pt(283).subsource = 'SS_DOUBLE';
pt(283).ndims     = '2';
pt(283).size      = '[]';

pt(284).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Encoder Initialization/Encoder Initializer';
pt(284).paramname = 'P3';
pt(284).class     = 'scalar';
pt(284).nrows     = 1;
pt(284).ncols     = 1;
pt(284).subsource = 'SS_DOUBLE';
pt(284).ndims     = '2';
pt(284).size      = '[]';

pt(285).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Encoder Initialization/Unwrapper';
pt(285).paramname = 'P2';
pt(285).class     = 'scalar';
pt(285).nrows     = 1;
pt(285).ncols     = 1;
pt(285).subsource = 'SS_DOUBLE';
pt(285).ndims     = '2';
pt(285).size      = '[]';

pt(286).blockname = 'Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc/Difference';
pt(286).paramname = 'Numerator';
pt(286).class     = 'vector';
pt(286).nrows     = 1;
pt(286).ncols     = 2;
pt(286).subsource = 'SS_DOUBLE';
pt(286).ndims     = '2';
pt(286).size      = '[]';

pt(287).blockname = 'Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc/Difference';
pt(287).paramname = 'Denominator';
pt(287).class     = 'vector';
pt(287).nrows     = 1;
pt(287).ncols     = 2;
pt(287).subsource = 'SS_DOUBLE';
pt(287).ndims     = '2';
pt(287).size      = '[]';

pt(288).blockname = 'Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc/Difference';
pt(288).paramname = 'InitialStates';
pt(288).class     = 'scalar';
pt(288).nrows     = 1;
pt(288).ncols     = 1;
pt(288).subsource = 'SS_DOUBLE';
pt(288).ndims     = '2';
pt(288).size      = '[]';

pt(289).blockname = 'Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc1/Difference';
pt(289).paramname = 'Numerator';
pt(289).class     = 'vector';
pt(289).nrows     = 1;
pt(289).ncols     = 2;
pt(289).subsource = 'SS_DOUBLE';
pt(289).ndims     = '2';
pt(289).size      = '[]';

pt(290).blockname = 'Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc1/Difference';
pt(290).paramname = 'Denominator';
pt(290).class     = 'vector';
pt(290).nrows     = 1;
pt(290).ncols     = 2;
pt(290).subsource = 'SS_DOUBLE';
pt(290).ndims     = '2';
pt(290).size      = '[]';

pt(291).blockname = 'Hardware I//O/Input/Rear Wheel Speed Processing/Speed Calc1/Difference';
pt(291).paramname = 'InitialStates';
pt(291).class     = 'scalar';
pt(291).nrows     = 1;
pt(291).ncols     = 1;
pt(291).subsource = 'SS_DOUBLE';
pt(291).ndims     = '2';
pt(291).size      = '[]';

pt(292).blockname = 'Hardware I//O/Input/Steering Motor Encoder Initialization/Left/Encoder Initializer';
pt(292).paramname = 'P2';
pt(292).class     = 'scalar';
pt(292).nrows     = 1;
pt(292).ncols     = 1;
pt(292).subsource = 'SS_DOUBLE';
pt(292).ndims     = '2';
pt(292).size      = '[]';

pt(293).blockname = 'Hardware I//O/Input/Steering Motor Encoder Initialization/Left/Encoder Initializer';
pt(293).paramname = 'P3';
pt(293).class     = 'scalar';
pt(293).nrows     = 1;
pt(293).ncols     = 1;
pt(293).subsource = 'SS_DOUBLE';
pt(293).ndims     = '2';
pt(293).size      = '[]';

pt(294).blockname = 'Hardware I//O/Input/Steering Motor Encoder Initialization/Left/Unwrapper';
pt(294).paramname = 'P2';
pt(294).class     = 'scalar';
pt(294).nrows     = 1;
pt(294).ncols     = 1;
pt(294).subsource = 'SS_DOUBLE';
pt(294).ndims     = '2';
pt(294).size      = '[]';

pt(295).blockname = 'Hardware I//O/Input/Steering Motor Encoder Initialization/Right/Encoder Initializer';
pt(295).paramname = 'P2';
pt(295).class     = 'scalar';
pt(295).nrows     = 1;
pt(295).ncols     = 1;
pt(295).subsource = 'SS_DOUBLE';
pt(295).ndims     = '2';
pt(295).size      = '[]';

pt(296).blockname = 'Hardware I//O/Input/Steering Motor Encoder Initialization/Right/Encoder Initializer';
pt(296).paramname = 'P3';
pt(296).class     = 'scalar';
pt(296).nrows     = 1;
pt(296).ncols     = 1;
pt(296).subsource = 'SS_DOUBLE';
pt(296).ndims     = '2';
pt(296).size      = '[]';

pt(297).blockname = 'Hardware I//O/Input/Steering Motor Encoder Initialization/Right/Unwrapper';
pt(297).paramname = 'P2';
pt(297).class     = 'scalar';
pt(297).nrows     = 1;
pt(297).ncols     = 1;
pt(297).subsource = 'SS_DOUBLE';
pt(297).ndims     = '2';
pt(297).size      = '[]';

pt(298).blockname = 'Hardware I//O/Input/VS330 GPS Data/Subsystem/Constant';
pt(298).paramname = 'Value';
pt(298).class     = 'scalar';
pt(298).nrows     = 1;
pt(298).ncols     = 1;
pt(298).subsource = 'SS_DOUBLE';
pt(298).ndims     = '2';
pt(298).size      = '[]';

pt(299).blockname = 'Hardware I//O/Input/VS330 GPS Data/Subsystem/WGS84 Eccentricity';
pt(299).paramname = 'Value';
pt(299).class     = 'scalar';
pt(299).nrows     = 1;
pt(299).ncols     = 1;
pt(299).subsource = 'SS_DOUBLE';
pt(299).ndims     = '2';
pt(299).size      = '[]';

pt(300).blockname = 'Hardware I//O/Input/VS330 GPS Data/Subsystem/WGS84 Semi-Major Axis';
pt(300).paramname = 'Value';
pt(300).class     = 'scalar';
pt(300).nrows     = 1;
pt(300).ncols     = 1;
pt(300).subsource = 'SS_DOUBLE';
pt(300).ndims     = '2';
pt(300).size      = '[]';

pt(301).blockname = 'Hardware I//O/Input/VS330 GPS Data/Subsystem/WGS84 Semi-Minor Axis';
pt(301).paramname = 'Value';
pt(301).class     = 'scalar';
pt(301).nrows     = 1;
pt(301).ncols     = 1;
pt(301).subsource = 'SS_DOUBLE';
pt(301).ndims     = '2';
pt(301).size      = '[]';

pt(302).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Left Steer Angle Look-up';
pt(302).paramname = 'Table';
pt(302).class     = 'vector';
pt(302).nrows     = 1;
pt(302).ncols     = 101;
pt(302).subsource = 'SS_DOUBLE';
pt(302).ndims     = '2';
pt(302).size      = '[]';

pt(303).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Left Steer Angle Look-up';
pt(303).paramname = 'BreakpointsForDimension1';
pt(303).class     = 'vector';
pt(303).nrows     = 1;
pt(303).ncols     = 101;
pt(303).subsource = 'SS_DOUBLE';
pt(303).ndims     = '2';
pt(303).size      = '[]';

pt(304).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Left Steer Angle Look-up1';
pt(304).paramname = 'Table';
pt(304).class     = 'vector';
pt(304).nrows     = 1;
pt(304).ncols     = 101;
pt(304).subsource = 'SS_DOUBLE';
pt(304).ndims     = '2';
pt(304).size      = '[]';

pt(305).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Left Steer Angle Look-up1';
pt(305).paramname = 'BreakpointsForDimension1';
pt(305).class     = 'vector';
pt(305).nrows     = 1;
pt(305).ncols     = 101;
pt(305).subsource = 'SS_DOUBLE';
pt(305).ndims     = '2';
pt(305).size      = '[]';

pt(306).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Right Steer Angle Look-up';
pt(306).paramname = 'Table';
pt(306).class     = 'vector';
pt(306).nrows     = 1;
pt(306).ncols     = 101;
pt(306).subsource = 'SS_DOUBLE';
pt(306).ndims     = '2';
pt(306).size      = '[]';

pt(307).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Right Steer Angle Look-up';
pt(307).paramname = 'BreakpointsForDimension1';
pt(307).class     = 'vector';
pt(307).nrows     = 1;
pt(307).ncols     = 101;
pt(307).subsource = 'SS_DOUBLE';
pt(307).ndims     = '2';
pt(307).size      = '[]';

pt(308).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Right Steer Angle Look-up1';
pt(308).paramname = 'Table';
pt(308).class     = 'vector';
pt(308).nrows     = 1;
pt(308).ncols     = 101;
pt(308).subsource = 'SS_DOUBLE';
pt(308).ndims     = '2';
pt(308).size      = '[]';

pt(309).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Steering Geometry Calculation/Right Steer Angle Look-up1';
pt(309).paramname = 'BreakpointsForDimension1';
pt(309).class     = 'vector';
pt(309).nrows     = 1;
pt(309).ncols     = 101;
pt(309).subsource = 'SS_DOUBLE';
pt(309).ndims     = '2';
pt(309).size      = '[]';

pt(310).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations/TSamp';
pt(310).paramname = 'WtEt';
pt(310).class     = 'scalar';
pt(310).nrows     = 1;
pt(310).ncols     = 1;
pt(310).subsource = 'SS_DOUBLE';
pt(310).ndims     = '2';
pt(310).size      = '[]';

pt(311).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Accelerations1/TSamp';
pt(311).paramname = 'WtEt';
pt(311).class     = 'scalar';
pt(311).nrows     = 1;
pt(311).ncols     = 1;
pt(311).subsource = 'SS_DOUBLE';
pt(311).ndims     = '2';
pt(311).size      = '[]';

pt(312).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities/TSamp';
pt(312).paramname = 'WtEt';
pt(312).class     = 'scalar';
pt(312).nrows     = 1;
pt(312).ncols     = 1;
pt(312).subsource = 'SS_DOUBLE';
pt(312).ndims     = '2';
pt(312).size      = '[]';

pt(313).blockname = 'Sensor Data and Estimation/Nonlinear Observer/Steering Torque Components/Wheel Velocities1/TSamp';
pt(313).paramname = 'WtEt';
pt(313).class     = 'scalar';
pt(313).nrows     = 1;
pt(313).ncols     = 1;
pt(313).subsource = 'SS_DOUBLE';
pt(313).ndims     = '2';
pt(313).size      = '[]';

pt(314).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Gain';
pt(314).paramname = 'Gain';
pt(314).class     = 'scalar';
pt(314).nrows     = 1;
pt(314).ncols     = 1;
pt(314).subsource = 'SS_DOUBLE';
pt(314).ndims     = '2';
pt(314).size      = '[]';

pt(315).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Position Filter';
pt(315).paramname = 'P1';
pt(315).class     = 'scalar';
pt(315).nrows     = 1;
pt(315).ncols     = 1;
pt(315).subsource = 'SS_DOUBLE';
pt(315).ndims     = '2';
pt(315).size      = '[]';

pt(316).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Position Filter';
pt(316).paramname = 'P2';
pt(316).class     = 'scalar';
pt(316).nrows     = 1;
pt(316).ncols     = 1;
pt(316).subsource = 'SS_DOUBLE';
pt(316).ndims     = '2';
pt(316).size      = '[]';

pt(317).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(317).paramname = 'P1';
pt(317).class     = 'vector';
pt(317).nrows     = 1;
pt(317).ncols     = 2;
pt(317).subsource = 'SS_DOUBLE';
pt(317).ndims     = '2';
pt(317).size      = '[]';

pt(318).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(318).paramname = 'P2';
pt(318).class     = 'vector';
pt(318).nrows     = 1;
pt(318).ncols     = 9;
pt(318).subsource = 'SS_DOUBLE';
pt(318).ndims     = '2';
pt(318).size      = '[]';

pt(319).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(319).paramname = 'P3';
pt(319).class     = 'scalar';
pt(319).nrows     = 1;
pt(319).ncols     = 1;
pt(319).subsource = 'SS_DOUBLE';
pt(319).ndims     = '2';
pt(319).size      = '[]';

pt(320).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(320).paramname = 'P4';
pt(320).class     = 'vector';
pt(320).nrows     = 1;
pt(320).ncols     = 4;
pt(320).subsource = 'SS_DOUBLE';
pt(320).ndims     = '2';
pt(320).size      = '[]';

pt(321).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(321).paramname = 'P5';
pt(321).class     = 'vector';
pt(321).nrows     = 1;
pt(321).ncols     = 2;
pt(321).subsource = 'SS_DOUBLE';
pt(321).ndims     = '2';
pt(321).size      = '[]';

pt(322).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(322).paramname = 'P6';
pt(322).class     = 'vector';
pt(322).nrows     = 1;
pt(322).ncols     = 16;
pt(322).subsource = 'SS_DOUBLE';
pt(322).ndims     = '2';
pt(322).size      = '[]';

pt(323).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(323).paramname = 'P7';
pt(323).class     = 'scalar';
pt(323).nrows     = 1;
pt(323).ncols     = 1;
pt(323).subsource = 'SS_DOUBLE';
pt(323).ndims     = '2';
pt(323).size      = '[]';

pt(324).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(324).paramname = 'P8';
pt(324).class     = 'scalar';
pt(324).nrows     = 1;
pt(324).ncols     = 1;
pt(324).subsource = 'SS_DOUBLE';
pt(324).ndims     = '2';
pt(324).size      = '[]';

pt(325).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(325).paramname = 'P9';
pt(325).class     = 'scalar';
pt(325).nrows     = 1;
pt(325).ncols     = 1;
pt(325).subsource = 'SS_DOUBLE';
pt(325).ndims     = '2';
pt(325).size      = '[]';

pt(326).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(326).paramname = 'P10';
pt(326).class     = 'scalar';
pt(326).nrows     = 1;
pt(326).ncols     = 1;
pt(326).subsource = 'SS_DOUBLE';
pt(326).ndims     = '2';
pt(326).size      = '[]';

pt(327).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(327).paramname = 'P11';
pt(327).class     = 'scalar';
pt(327).nrows     = 1;
pt(327).ncols     = 1;
pt(327).subsource = 'SS_DOUBLE';
pt(327).ndims     = '2';
pt(327).size      = '[]';

pt(328).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(328).paramname = 'P12';
pt(328).class     = 'scalar';
pt(328).nrows     = 1;
pt(328).ncols     = 1;
pt(328).subsource = 'SS_DOUBLE';
pt(328).ndims     = '2';
pt(328).size      = '[]';

pt(329).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(329).paramname = 'P13';
pt(329).class     = 'scalar';
pt(329).nrows     = 1;
pt(329).ncols     = 1;
pt(329).subsource = 'SS_DOUBLE';
pt(329).ndims     = '2';
pt(329).size      = '[]';

pt(330).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(330).paramname = 'P14';
pt(330).class     = 'scalar';
pt(330).nrows     = 1;
pt(330).ncols     = 1;
pt(330).subsource = 'SS_DOUBLE';
pt(330).ndims     = '2';
pt(330).size      = '[]';

pt(331).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(331).paramname = 'P15';
pt(331).class     = 'scalar';
pt(331).nrows     = 1;
pt(331).ncols     = 1;
pt(331).subsource = 'SS_DOUBLE';
pt(331).ndims     = '2';
pt(331).size      = '[]';

pt(332).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(332).paramname = 'P16';
pt(332).class     = 'scalar';
pt(332).nrows     = 1;
pt(332).ncols     = 1;
pt(332).subsource = 'SS_DOUBLE';
pt(332).ndims     = '2';
pt(332).size      = '[]';

pt(333).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(333).paramname = 'P17';
pt(333).class     = 'scalar';
pt(333).nrows     = 1;
pt(333).ncols     = 1;
pt(333).subsource = 'SS_DOUBLE';
pt(333).ndims     = '2';
pt(333).size      = '[]';

pt(334).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(334).paramname = 'P18';
pt(334).class     = 'scalar';
pt(334).nrows     = 1;
pt(334).ncols     = 1;
pt(334).subsource = 'SS_DOUBLE';
pt(334).ndims     = '2';
pt(334).size      = '[]';

pt(335).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(335).paramname = 'P19';
pt(335).class     = 'scalar';
pt(335).nrows     = 1;
pt(335).ncols     = 1;
pt(335).subsource = 'SS_DOUBLE';
pt(335).ndims     = '2';
pt(335).size      = '[]';

pt(336).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(336).paramname = 'P20';
pt(336).class     = 'scalar';
pt(336).nrows     = 1;
pt(336).ncols     = 1;
pt(336).subsource = 'SS_DOUBLE';
pt(336).ndims     = '2';
pt(336).size      = '[]';

pt(337).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(337).paramname = 'P21';
pt(337).class     = 'scalar';
pt(337).nrows     = 1;
pt(337).ncols     = 1;
pt(337).subsource = 'SS_DOUBLE';
pt(337).ndims     = '2';
pt(337).size      = '[]';

pt(338).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(338).paramname = 'P22';
pt(338).class     = 'scalar';
pt(338).nrows     = 1;
pt(338).ncols     = 1;
pt(338).subsource = 'SS_DOUBLE';
pt(338).ndims     = '2';
pt(338).size      = '[]';

pt(339).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(339).paramname = 'P23';
pt(339).class     = 'scalar';
pt(339).nrows     = 1;
pt(339).ncols     = 1;
pt(339).subsource = 'SS_DOUBLE';
pt(339).ndims     = '2';
pt(339).size      = '[]';

pt(340).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(340).paramname = 'P24';
pt(340).class     = 'scalar';
pt(340).nrows     = 1;
pt(340).ncols     = 1;
pt(340).subsource = 'SS_DOUBLE';
pt(340).ndims     = '2';
pt(340).size      = '[]';

pt(341).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(341).paramname = 'P25';
pt(341).class     = 'scalar';
pt(341).nrows     = 1;
pt(341).ncols     = 1;
pt(341).subsource = 'SS_DOUBLE';
pt(341).ndims     = '2';
pt(341).size      = '[]';

pt(342).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(342).paramname = 'P26';
pt(342).class     = 'scalar';
pt(342).nrows     = 1;
pt(342).ncols     = 1;
pt(342).subsource = 'SS_DOUBLE';
pt(342).ndims     = '2';
pt(342).size      = '[]';

pt(343).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(343).paramname = 'P27';
pt(343).class     = 'scalar';
pt(343).nrows     = 1;
pt(343).ncols     = 1;
pt(343).subsource = 'SS_DOUBLE';
pt(343).ndims     = '2';
pt(343).size      = '[]';

pt(344).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(344).paramname = 'P28';
pt(344).class     = 'scalar';
pt(344).nrows     = 1;
pt(344).ncols     = 1;
pt(344).subsource = 'SS_DOUBLE';
pt(344).ndims     = '2';
pt(344).size      = '[]';

pt(345).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(345).paramname = 'P29';
pt(345).class     = 'scalar';
pt(345).nrows     = 1;
pt(345).ncols     = 1;
pt(345).subsource = 'SS_DOUBLE';
pt(345).ndims     = '2';
pt(345).size      = '[]';

pt(346).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(346).paramname = 'P30';
pt(346).class     = 'scalar';
pt(346).nrows     = 1;
pt(346).ncols     = 1;
pt(346).subsource = 'SS_DOUBLE';
pt(346).ndims     = '2';
pt(346).size      = '[]';

pt(347).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Super Filter!';
pt(347).paramname = 'P31';
pt(347).class     = 'scalar';
pt(347).nrows     = 1;
pt(347).ncols     = 1;
pt(347).subsource = 'SS_DOUBLE';
pt(347).ndims     = '2';
pt(347).size      = '[]';

pt(348).blockname = 'User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay';
pt(348).paramname = 'InitialCondition';
pt(348).class     = 'scalar';
pt(348).nrows     = 1;
pt(348).ncols     = 1;
pt(348).subsource = 'SS_DOUBLE';
pt(348).ndims     = '2';
pt(348).size      = '[]';

pt(349).blockname = 'User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay1';
pt(349).paramname = 'InitialCondition';
pt(349).class     = 'scalar';
pt(349).nrows     = 1;
pt(349).ncols     = 1;
pt(349).subsource = 'SS_DOUBLE';
pt(349).ndims     = '2';
pt(349).size      = '[]';

pt(350).blockname = 'User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay2';
pt(350).paramname = 'InitialCondition';
pt(350).class     = 'scalar';
pt(350).nrows     = 1;
pt(350).ncols     = 1;
pt(350).subsource = 'SS_DOUBLE';
pt(350).ndims     = '2';
pt(350).size      = '[]';

pt(351).blockname = 'User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay3';
pt(351).paramname = 'InitialCondition';
pt(351).class     = 'scalar';
pt(351).nrows     = 1;
pt(351).ncols     = 1;
pt(351).subsource = 'SS_DOUBLE';
pt(351).ndims     = '2';
pt(351).size      = '[]';

pt(352).blockname = 'User Controllers/Cruise Control/Enable Logic/Delay/Unit Delay4';
pt(352).paramname = 'InitialCondition';
pt(352).class     = 'scalar';
pt(352).nrows     = 1;
pt(352).ncols     = 1;
pt(352).subsource = 'SS_DOUBLE';
pt(352).ndims     = '2';
pt(352).size      = '[]';

pt(353).blockname = 'User Controllers/Cruise Control/Enable Logic/Latch1/Out1';
pt(353).paramname = 'InitialOutput';
pt(353).class     = 'scalar';
pt(353).nrows     = 1;
pt(353).ncols     = 1;
pt(353).subsource = 'SS_DOUBLE';
pt(353).ndims     = '2';
pt(353).size      = '[]';

pt(354).blockname = 'User Controllers/Steering Controller/HAL-9000/Chirp Signal/Gain';
pt(354).paramname = 'Gain';
pt(354).class     = 'scalar';
pt(354).nrows     = 1;
pt(354).ncols     = 1;
pt(354).subsource = 'SS_DOUBLE';
pt(354).ndims     = '2';
pt(354).size      = '[]';

pt(355).blockname = 'User Controllers/Steering Controller/HAL-9000/Chirp Signal/Saturation';
pt(355).paramname = 'UpperLimit';
pt(355).class     = 'scalar';
pt(355).nrows     = 1;
pt(355).ncols     = 1;
pt(355).subsource = 'SS_DOUBLE';
pt(355).ndims     = '2';
pt(355).size      = '[]';

pt(356).blockname = 'User Controllers/Steering Controller/HAL-9000/Chirp Signal/Saturation';
pt(356).paramname = 'LowerLimit';
pt(356).class     = 'scalar';
pt(356).nrows     = 1;
pt(356).ncols     = 1;
pt(356).subsource = 'SS_DOUBLE';
pt(356).ndims     = '2';
pt(356).size      = '[]';

pt(357).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp One';
pt(357).paramname = 'f1';
pt(357).class     = 'scalar';
pt(357).nrows     = 1;
pt(357).ncols     = 1;
pt(357).subsource = 'SS_DOUBLE';
pt(357).ndims     = '2';
pt(357).size      = '[]';

pt(358).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp One';
pt(358).paramname = 'T';
pt(358).class     = 'scalar';
pt(358).nrows     = 1;
pt(358).ncols     = 1;
pt(358).subsource = 'SS_DOUBLE';
pt(358).ndims     = '2';
pt(358).size      = '[]';

pt(359).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp One';
pt(359).paramname = 'f2';
pt(359).class     = 'scalar';
pt(359).nrows     = 1;
pt(359).ncols     = 1;
pt(359).subsource = 'SS_DOUBLE';
pt(359).ndims     = '2';
pt(359).size      = '[]';

pt(360).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp Two';
pt(360).paramname = 'f1';
pt(360).class     = 'scalar';
pt(360).nrows     = 1;
pt(360).ncols     = 1;
pt(360).subsource = 'SS_DOUBLE';
pt(360).ndims     = '2';
pt(360).size      = '[]';

pt(361).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp Two';
pt(361).paramname = 'T';
pt(361).class     = 'scalar';
pt(361).nrows     = 1;
pt(361).ncols     = 1;
pt(361).subsource = 'SS_DOUBLE';
pt(361).ndims     = '2';
pt(361).size      = '[]';

pt(362).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp Two';
pt(362).paramname = 'f2';
pt(362).class     = 'scalar';
pt(362).nrows     = 1;
pt(362).ncols     = 1;
pt(362).subsource = 'SS_DOUBLE';
pt(362).ndims     = '2';
pt(362).size      = '[]';

pt(363).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/initialFreq';
pt(363).paramname = 'Value';
pt(363).class     = 'scalar';
pt(363).nrows     = 1;
pt(363).ncols     = 1;
pt(363).subsource = 'SS_DOUBLE';
pt(363).ndims     = '2';
pt(363).size      = '[]';

pt(364).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Switch';
pt(364).paramname = 'Threshold';
pt(364).class     = 'scalar';
pt(364).nrows     = 1;
pt(364).ncols     = 1;
pt(364).subsource = 'SS_DOUBLE';
pt(364).ndims     = '2';
pt(364).size      = '[]';

pt(365).blockname = 'User Controllers/Steering Controller/HAL-9000/Double-step/Left Pulse';
pt(365).paramname = 'uplimit';
pt(365).class     = 'scalar';
pt(365).nrows     = 1;
pt(365).ncols     = 1;
pt(365).subsource = 'SS_DOUBLE';
pt(365).ndims     = '2';
pt(365).size      = '[]';

pt(366).blockname = 'User Controllers/Steering Controller/HAL-9000/Double-step/Left Pulse';
pt(366).paramname = 'lowlimit';
pt(366).class     = 'scalar';
pt(366).nrows     = 1;
pt(366).ncols     = 1;
pt(366).subsource = 'SS_DOUBLE';
pt(366).ndims     = '2';
pt(366).size      = '[]';

pt(367).blockname = 'User Controllers/Steering Controller/HAL-9000/Double-step/Right Pulse';
pt(367).paramname = 'uplimit';
pt(367).class     = 'scalar';
pt(367).nrows     = 1;
pt(367).ncols     = 1;
pt(367).subsource = 'SS_DOUBLE';
pt(367).ndims     = '2';
pt(367).size      = '[]';

pt(368).blockname = 'User Controllers/Steering Controller/HAL-9000/Double-step/Right Pulse';
pt(368).paramname = 'lowlimit';
pt(368).class     = 'scalar';
pt(368).nrows     = 1;
pt(368).ncols     = 1;
pt(368).subsource = 'SS_DOUBLE';
pt(368).ndims     = '2';
pt(368).size      = '[]';

pt(369).blockname = 'User Controllers/Steering Controller/HAL-9000/Latch/Out1';
pt(369).paramname = 'InitialOutput';
pt(369).class     = 'scalar';
pt(369).nrows     = 1;
pt(369).ncols     = 1;
pt(369).subsource = 'SS_DOUBLE';
pt(369).ndims     = '2';
pt(369).size      = '[]';

pt(370).blockname = 'User Controllers/Steering Controller/HAL-9000/Ramp/Slope';
pt(370).paramname = 'Gain';
pt(370).class     = 'scalar';
pt(370).nrows     = 1;
pt(370).ncols     = 1;
pt(370).subsource = 'SS_DOUBLE';
pt(370).ndims     = '2';
pt(370).size      = '[]';

pt(371).blockname = 'User Controllers/Steering Controller/HAL-9000/Ramp/Saturation';
pt(371).paramname = 'UpperLimit';
pt(371).class     = 'scalar';
pt(371).nrows     = 1;
pt(371).ncols     = 1;
pt(371).subsource = 'SS_DOUBLE';
pt(371).ndims     = '2';
pt(371).size      = '[]';

pt(372).blockname = 'User Controllers/Steering Controller/HAL-9000/Ramp/Saturation';
pt(372).paramname = 'LowerLimit';
pt(372).class     = 'scalar';
pt(372).nrows     = 1;
pt(372).ncols     = 1;
pt(372).subsource = 'SS_DOUBLE';
pt(372).ndims     = '2';
pt(372).size      = '[]';

pt(373).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Memory3';
pt(373).paramname = 'X0';
pt(373).class     = 'scalar';
pt(373).nrows     = 1;
pt(373).ncols     = 1;
pt(373).subsource = 'SS_DOUBLE';
pt(373).ndims     = '2';
pt(373).size      = '[]';

pt(374).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Enable Timer/Constant';
pt(374).paramname = 'Value';
pt(374).class     = 'scalar';
pt(374).nrows     = 1;
pt(374).ncols     = 1;
pt(374).subsource = 'SS_DOUBLE';
pt(374).ndims     = '2';
pt(374).size      = '[]';

pt(375).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Memory';
pt(375).paramname = 'X0';
pt(375).class     = 'scalar';
pt(375).nrows     = 1;
pt(375).ncols     = 1;
pt(375).subsource = 'SS_DOUBLE';
pt(375).ndims     = '2';
pt(375).size      = '[]';

pt(376).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Memory1';
pt(376).paramname = 'X0';
pt(376).class     = 'scalar';
pt(376).nrows     = 1;
pt(376).ncols     = 1;
pt(376).subsource = 'SS_DOUBLE';
pt(376).ndims     = '2';
pt(376).size      = '[]';

pt(377).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Memory2';
pt(377).paramname = 'X0';
pt(377).class     = 'scalar';
pt(377).nrows     = 1;
pt(377).ncols     = 1;
pt(377).subsource = 'SS_DOUBLE';
pt(377).ndims     = '2';
pt(377).size      = '[]';

pt(378).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Switch Over/Switch';
pt(378).paramname = 'Threshold';
pt(378).class     = 'scalar';
pt(378).nrows     = 1;
pt(378).ncols     = 1;
pt(378).subsource = 'SS_DOUBLE';
pt(378).ndims     = '2';
pt(378).size      = '[]';

pt(379).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Switch Over/Switch1';
pt(379).paramname = 'Threshold';
pt(379).class     = 'scalar';
pt(379).nrows     = 1;
pt(379).ncols     = 1;
pt(379).subsource = 'SS_DOUBLE';
pt(379).ndims     = '2';
pt(379).size      = '[]';

pt(380).blockname = 'Hardware I//O/Input/VS330 GPS Data/Subsystem/Degrees to Radians/Gain1';
pt(380).paramname = 'Gain';
pt(380).class     = 'scalar';
pt(380).nrows     = 1;
pt(380).ncols     = 1;
pt(380).subsource = 'SS_DOUBLE';
pt(380).ndims     = '2';
pt(380).size      = '[]';

pt(381).blockname = 'Hardware I//O/Input/VS330 GPS Data/Subsystem/Degrees to Radians1/Gain1';
pt(381).paramname = 'Gain';
pt(381).class     = 'scalar';
pt(381).nrows     = 1;
pt(381).ncols     = 1;
pt(381).subsource = 'SS_DOUBLE';
pt(381).ndims     = '2';
pt(381).size      = '[]';

pt(382).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/ax Bias';
pt(382).paramname = 'Value';
pt(382).class     = 'scalar';
pt(382).nrows     = 1;
pt(382).ncols     = 1;
pt(382).subsource = 'SS_DOUBLE';
pt(382).ndims     = '2';
pt(382).size      = '[]';

pt(383).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/ay Bias';
pt(383).paramname = 'Value';
pt(383).class     = 'scalar';
pt(383).nrows     = 1;
pt(383).ncols     = 1;
pt(383).subsource = 'SS_DOUBLE';
pt(383).ndims     = '2';
pt(383).size      = '[]';

pt(384).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/ax sf';
pt(384).paramname = 'Gain';
pt(384).class     = 'scalar';
pt(384).nrows     = 1;
pt(384).ncols     = 1;
pt(384).subsource = 'SS_DOUBLE';
pt(384).ndims     = '2';
pt(384).size      = '[]';

pt(385).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/ay sf';
pt(385).paramname = 'Gain';
pt(385).class     = 'scalar';
pt(385).nrows     = 1;
pt(385).ncols     = 1;
pt(385).subsource = 'SS_DOUBLE';
pt(385).ndims     = '2';
pt(385).size      = '[]';

pt(386).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/cross coupling from ax';
pt(386).paramname = 'Gain';
pt(386).class     = 'scalar';
pt(386).nrows     = 1;
pt(386).ncols     = 1;
pt(386).subsource = 'SS_DOUBLE';
pt(386).ndims     = '2';
pt(386).size      = '[]';

pt(387).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/cross coupling from ay';
pt(387).paramname = 'Gain';
pt(387).class     = 'scalar';
pt(387).nrows     = 1;
pt(387).ncols     = 1;
pt(387).subsource = 'SS_DOUBLE';
pt(387).ndims     = '2';
pt(387).size      = '[]';

pt(388).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Accelerometer Pre-Processing/Memory';
pt(388).paramname = 'X0';
pt(388).class     = 'scalar';
pt(388).nrows     = 1;
pt(388).ncols     = 1;
pt(388).subsource = 'SS_DOUBLE';
pt(388).ndims     = '2';
pt(388).size      = '[]';

pt(389).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/Compare To Constant';
pt(389).paramname = 'const';
pt(389).class     = 'scalar';
pt(389).nrows     = 1;
pt(389).ncols     = 1;
pt(389).subsource = 'SS_UINT16';
pt(389).ndims     = '2';
pt(389).size      = '[]';

pt(390).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/GPS Roll Offset';
pt(390).paramname = 'Value';
pt(390).class     = 'scalar';
pt(390).nrows     = 1;
pt(390).ncols     = 1;
pt(390).subsource = 'SS_DOUBLE';
pt(390).ndims     = '2';
pt(390).size      = '[]';

pt(391).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/GPS Yaw Offset';
pt(391).paramname = 'Value';
pt(391).class     = 'scalar';
pt(391).nrows     = 1;
pt(391).ncols     = 1;
pt(391).subsource = 'SS_DOUBLE';
pt(391).ndims     = '2';
pt(391).size      = '[]';

pt(392).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/bad';
pt(392).paramname = 'Value';
pt(392).class     = 'scalar';
pt(392).nrows     = 1;
pt(392).ncols     = 1;
pt(392).subsource = 'SS_DOUBLE';
pt(392).ndims     = '2';
pt(392).size      = '[]';

pt(393).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/bad1';
pt(393).paramname = 'Value';
pt(393).class     = 'scalar';
pt(393).nrows     = 1;
pt(393).ncols     = 1;
pt(393).subsource = 'SS_DOUBLE';
pt(393).ndims     = '2';
pt(393).size      = '[]';

pt(394).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/good';
pt(394).paramname = 'Value';
pt(394).class     = 'scalar';
pt(394).nrows     = 1;
pt(394).ncols     = 1;
pt(394).subsource = 'SS_DOUBLE';
pt(394).ndims     = '2';
pt(394).size      = '[]';

pt(395).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/good1';
pt(395).paramname = 'Value';
pt(395).class     = 'scalar';
pt(395).nrows     = 1;
pt(395).ncols     = 1;
pt(395).subsource = 'SS_DOUBLE';
pt(395).ndims     = '2';
pt(395).size      = '[]';

pt(396).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/obsolete1';
pt(396).paramname = 'Value';
pt(396).class     = 'scalar';
pt(396).nrows     = 1;
pt(396).ncols     = 1;
pt(396).subsource = 'SS_DOUBLE';
pt(396).ndims     = '2';
pt(396).size      = '[]';

pt(397).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/obsolete2';
pt(397).paramname = 'Value';
pt(397).class     = 'scalar';
pt(397).nrows     = 1;
pt(397).ncols     = 1;
pt(397).subsource = 'SS_DOUBLE';
pt(397).ndims     = '2';
pt(397).size      = '[]';

pt(398).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/Gain';
pt(398).paramname = 'Gain';
pt(398).class     = 'scalar';
pt(398).nrows     = 1;
pt(398).ncols     = 1;
pt(398).subsource = 'SS_DOUBLE';
pt(398).ndims     = '2';
pt(398).size      = '[]';

pt(399).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/Bitwise Operator';
pt(399).paramname = 'BitMask';
pt(399).class     = 'scalar';
pt(399).nrows     = 1;
pt(399).ncols     = 1;
pt(399).subsource = 'SS_UINT16';
pt(399).ndims     = '2';
pt(399).size      = '[]';

pt(400).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/Delay function';
pt(400).paramname = 'P1';
pt(400).class     = 'scalar';
pt(400).nrows     = 1;
pt(400).ncols     = 1;
pt(400).subsource = 'SS_DOUBLE';
pt(400).ndims     = '2';
pt(400).size      = '[]';

pt(401).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/Delay function';
pt(401).paramname = 'P2';
pt(401).class     = 'scalar';
pt(401).nrows     = 1;
pt(401).ncols     = 1;
pt(401).subsource = 'SS_DOUBLE';
pt(401).ndims     = '2';
pt(401).size      = '[]';

pt(402).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/GPS Pre-Processing/Switch2';
pt(402).paramname = 'Threshold';
pt(402).class     = 'scalar';
pt(402).nrows     = 1;
pt(402).ncols     = 1;
pt(402).subsource = 'SS_DOUBLE';
pt(402).ndims     = '2';
pt(402).size      = '[]';

pt(403).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/Roll Gyro Bias';
pt(403).paramname = 'Value';
pt(403).class     = 'scalar';
pt(403).nrows     = 1;
pt(403).ncols     = 1;
pt(403).subsource = 'SS_DOUBLE';
pt(403).ndims     = '2';
pt(403).size      = '[]';

pt(404).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/Yaw Gyro Bias';
pt(404).paramname = 'Value';
pt(404).class     = 'scalar';
pt(404).nrows     = 1;
pt(404).ncols     = 1;
pt(404).subsource = 'SS_DOUBLE';
pt(404).ndims     = '2';
pt(404).size      = '[]';

pt(405).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/cross coupling from roll';
pt(405).paramname = 'Gain';
pt(405).class     = 'scalar';
pt(405).nrows     = 1;
pt(405).ncols     = 1;
pt(405).subsource = 'SS_DOUBLE';
pt(405).ndims     = '2';
pt(405).size      = '[]';

pt(406).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/cross coupling from yaw';
pt(406).paramname = 'Gain';
pt(406).class     = 'scalar';
pt(406).nrows     = 1;
pt(406).ncols     = 1;
pt(406).subsource = 'SS_DOUBLE';
pt(406).ndims     = '2';
pt(406).size      = '[]';

pt(407).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/roll rate sf';
pt(407).paramname = 'Gain';
pt(407).class     = 'scalar';
pt(407).nrows     = 1;
pt(407).ncols     = 1;
pt(407).subsource = 'SS_DOUBLE';
pt(407).ndims     = '2';
pt(407).size      = '[]';

pt(408).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/yaw rate sf';
pt(408).paramname = 'Gain';
pt(408).class     = 'scalar';
pt(408).nrows     = 1;
pt(408).ncols     = 1;
pt(408).subsource = 'SS_DOUBLE';
pt(408).ndims     = '2';
pt(408).size      = '[]';

pt(409).blockname = 'Sensor Data and Estimation/Vehicle State Estimator/state estimation/Super Filter/Roll//Yaw Gyro Pre-processing/Memory';
pt(409).paramname = 'X0';
pt(409).class     = 'scalar';
pt(409).nrows     = 1;
pt(409).ncols     = 1;
pt(409).subsource = 'SS_DOUBLE';
pt(409).ndims     = '2';
pt(409).size      = '[]';

pt(410).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp One/Gain';
pt(410).paramname = 'Gain';
pt(410).class     = 'scalar';
pt(410).nrows     = 1;
pt(410).ncols     = 1;
pt(410).subsource = 'SS_DOUBLE';
pt(410).ndims     = '2';
pt(410).size      = '[]';

pt(411).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp One/Saturation';
pt(411).paramname = 'UpperLimit';
pt(411).class     = 'scalar';
pt(411).nrows     = 1;
pt(411).ncols     = 1;
pt(411).subsource = 'SS_DOUBLE';
pt(411).ndims     = '2';
pt(411).size      = '[]';

pt(412).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp One/Saturation';
pt(412).paramname = 'LowerLimit';
pt(412).class     = 'scalar';
pt(412).nrows     = 1;
pt(412).ncols     = 1;
pt(412).subsource = 'SS_DOUBLE';
pt(412).ndims     = '2';
pt(412).size      = '[]';

pt(413).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp Two/Gain';
pt(413).paramname = 'Gain';
pt(413).class     = 'scalar';
pt(413).nrows     = 1;
pt(413).ncols     = 1;
pt(413).subsource = 'SS_DOUBLE';
pt(413).ndims     = '2';
pt(413).size      = '[]';

pt(414).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp Two/Saturation';
pt(414).paramname = 'UpperLimit';
pt(414).class     = 'scalar';
pt(414).nrows     = 1;
pt(414).ncols     = 1;
pt(414).subsource = 'SS_DOUBLE';
pt(414).ndims     = '2';
pt(414).size      = '[]';

pt(415).blockname = 'User Controllers/Steering Controller/HAL-9000/Double chirp/Chirp Two/Saturation';
pt(415).paramname = 'LowerLimit';
pt(415).class     = 'scalar';
pt(415).nrows     = 1;
pt(415).ncols     = 1;
pt(415).subsource = 'SS_DOUBLE';
pt(415).ndims     = '2';
pt(415).size      = '[]';

pt(416).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Change Detect/Constant';
pt(416).paramname = 'Value';
pt(416).class     = 'scalar';
pt(416).nrows     = 1;
pt(416).ncols     = 1;
pt(416).subsource = 'SS_DOUBLE';
pt(416).ndims     = '2';
pt(416).size      = '[]';

pt(417).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Latch/Constant';
pt(417).paramname = 'Value';
pt(417).class     = 'scalar';
pt(417).nrows     = 1;
pt(417).ncols     = 1;
pt(417).subsource = 'SS_DOUBLE';
pt(417).ndims     = '2';
pt(417).size      = '[]';

pt(418).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Latch1/Constant';
pt(418).paramname = 'Value';
pt(418).class     = 'scalar';
pt(418).nrows     = 1;
pt(418).ncols     = 1;
pt(418).subsource = 'SS_DOUBLE';
pt(418).ndims     = '2';
pt(418).size      = '[]';

pt(419).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter/Constant';
pt(419).paramname = 'Value';
pt(419).class     = 'scalar';
pt(419).nrows     = 1;
pt(419).ncols     = 1;
pt(419).subsource = 'SS_DOUBLE';
pt(419).ndims     = '2';
pt(419).size      = '[]';

pt(420).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter/Constant1';
pt(420).paramname = 'Value';
pt(420).class     = 'scalar';
pt(420).nrows     = 1;
pt(420).ncols     = 1;
pt(420).subsource = 'SS_DOUBLE';
pt(420).ndims     = '2';
pt(420).size      = '[]';

pt(421).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter/Discrete-Time Integrator';
pt(421).paramname = 'gainval';
pt(421).class     = 'scalar';
pt(421).nrows     = 1;
pt(421).ncols     = 1;
pt(421).subsource = 'SS_DOUBLE';
pt(421).ndims     = '2';
pt(421).size      = '[]';

pt(422).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter/Discrete-Time Integrator';
pt(422).paramname = 'InitialCondition';
pt(422).class     = 'scalar';
pt(422).nrows     = 1;
pt(422).ncols     = 1;
pt(422).subsource = 'SS_DOUBLE';
pt(422).ndims     = '2';
pt(422).size      = '[]';

pt(423).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter/Discrete-Time Integrator';
pt(423).paramname = 'UpperSaturationLimit';
pt(423).class     = 'scalar';
pt(423).nrows     = 1;
pt(423).ncols     = 1;
pt(423).subsource = 'SS_DOUBLE';
pt(423).ndims     = '2';
pt(423).size      = '[]';

pt(424).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter/Discrete-Time Integrator';
pt(424).paramname = 'LowerSaturationLimit';
pt(424).class     = 'scalar';
pt(424).nrows     = 1;
pt(424).ncols     = 1;
pt(424).subsource = 'SS_DOUBLE';
pt(424).ndims     = '2';
pt(424).size      = '[]';

pt(425).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter1/Constant';
pt(425).paramname = 'Value';
pt(425).class     = 'scalar';
pt(425).nrows     = 1;
pt(425).ncols     = 1;
pt(425).subsource = 'SS_DOUBLE';
pt(425).ndims     = '2';
pt(425).size      = '[]';

pt(426).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter1/Constant1';
pt(426).paramname = 'Value';
pt(426).class     = 'scalar';
pt(426).nrows     = 1;
pt(426).ncols     = 1;
pt(426).subsource = 'SS_DOUBLE';
pt(426).ndims     = '2';
pt(426).size      = '[]';

pt(427).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter1/Discrete-Time Integrator';
pt(427).paramname = 'gainval';
pt(427).class     = 'scalar';
pt(427).nrows     = 1;
pt(427).ncols     = 1;
pt(427).subsource = 'SS_DOUBLE';
pt(427).ndims     = '2';
pt(427).size      = '[]';

pt(428).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter1/Discrete-Time Integrator';
pt(428).paramname = 'InitialCondition';
pt(428).class     = 'scalar';
pt(428).nrows     = 1;
pt(428).ncols     = 1;
pt(428).subsource = 'SS_DOUBLE';
pt(428).ndims     = '2';
pt(428).size      = '[]';

pt(429).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter1/Discrete-Time Integrator';
pt(429).paramname = 'UpperSaturationLimit';
pt(429).class     = 'scalar';
pt(429).nrows     = 1;
pt(429).ncols     = 1;
pt(429).subsource = 'SS_DOUBLE';
pt(429).ndims     = '2';
pt(429).size      = '[]';

pt(430).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Counter1/Discrete-Time Integrator';
pt(430).paramname = 'LowerSaturationLimit';
pt(430).class     = 'scalar';
pt(430).nrows     = 1;
pt(430).ncols     = 1;
pt(430).subsource = 'SS_DOUBLE';
pt(430).ndims     = '2';
pt(430).size      = '[]';

pt(431).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Enabled Subsystem/Out1';
pt(431).paramname = 'InitialOutput';
pt(431).class     = 'scalar';
pt(431).nrows     = 1;
pt(431).ncols     = 1;
pt(431).subsource = 'SS_DOUBLE';
pt(431).ndims     = '2';
pt(431).size      = '[]';

pt(432).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Enabled Subsystem1/Out1';
pt(432).paramname = 'InitialOutput';
pt(432).class     = 'scalar';
pt(432).nrows     = 1;
pt(432).ncols     = 1;
pt(432).subsource = 'SS_DOUBLE';
pt(432).ndims     = '2';
pt(432).size      = '[]';

pt(433).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Nearly the Same/Constant';
pt(433).paramname = 'Value';
pt(433).class     = 'scalar';
pt(433).nrows     = 1;
pt(433).ncols     = 1;
pt(433).subsource = 'SS_DOUBLE';
pt(433).ndims     = '2';
pt(433).size      = '[]';

pt(434).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Nearly the Same1/Constant';
pt(434).paramname = 'Value';
pt(434).class     = 'scalar';
pt(434).nrows     = 1;
pt(434).ncols     = 1;
pt(434).subsource = 'SS_DOUBLE';
pt(434).ndims     = '2';
pt(434).size      = '[]';

pt(435).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Nearly the Same2/Constant';
pt(435).paramname = 'Value';
pt(435).class     = 'scalar';
pt(435).nrows     = 1;
pt(435).ncols     = 1;
pt(435).subsource = 'SS_DOUBLE';
pt(435).ndims     = '2';
pt(435).size      = '[]';

pt(436).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Fault Detection1/Nearly the Same3/Constant';
pt(436).paramname = 'Value';
pt(436).class     = 'scalar';
pt(436).nrows     = 1;
pt(436).ncols     = 1;
pt(436).subsource = 'SS_DOUBLE';
pt(436).ndims     = '2';
pt(436).size      = '[]';

pt(437).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Latch/Enabled Subsystem/Out1';
pt(437).paramname = 'InitialOutput';
pt(437).class     = 'scalar';
pt(437).nrows     = 1;
pt(437).ncols     = 1;
pt(437).subsource = 'SS_DOUBLE';
pt(437).ndims     = '2';
pt(437).size      = '[]';

pt(438).blockname = 'Hardware I//O/Input/Handwheel Scaling & Initialization/Handwheel Sensor Fault Detection/Descision Logic/Latch1/Enabled Subsystem/Out1';
pt(438).paramname = 'InitialOutput';
pt(438).class     = 'scalar';
pt(438).nrows     = 1;
pt(438).ncols     = 1;
pt(438).subsource = 'SS_DOUBLE';
pt(438).ndims     = '2';
pt(438).size      = '[]';

pt(439).blockname = '';
pt(439).paramname = 'Ts';
pt(439).class     = 'scalar';
pt(439).nrows     = 1;
pt(439).ncols     = 1;
pt(439).subsource = 'SS_DOUBLE';
pt(439).ndims     = '2';
pt(439).size      = '[]';

function len = getlenPT
len = 439;

