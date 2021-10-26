function residuals = SuspAngles(testAngles,Forces)

    gamma = testAngles(1); theta = testAngles(2);
    Fmag = norm(Forces);
    
    residuals = [Forces(1)/Fmag + sin(theta);
        Forces(2)/Fmag - cos(theta)*sin(gamma);
        Forces(3)/Fmag - cos(theta)*cos(gamma)];
end