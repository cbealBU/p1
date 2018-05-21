%Lanekeeping Parameters
Cfl = param.fl.C;
Cfr = param.fr.C;

Cf = Cfl + Cfr;

Crl = param.rl.C;
Crr = param.rr.C;

Cr = Crl + Crr;

pfk=3500;
xla=(Cf+Cr)/2/pfk;

lkparams.pfk = pfk;
lkparams.xla = xla;