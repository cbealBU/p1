/*	7/09/02 JPS
 * File : lat_err_yaw.c
 *
 *	[error, seg, u, angle]=lat_err_yaw(segment, x, y, heading angle)
 *
 * Abstract:
 *       This S function calculates lateral position and heading
 *		error based on current GPS position and velocity data.
 *      It has the following basic outline:
 *			1.  Take-in (or type in) map coefficients.  Map is
 *			polynomial segments found using mapfunclosed or similar
 *			2.For current and next segment of the map, find error
 *			using projection theorem.
 *				a.  For current and next segment, solve polynomial
 *				using mueller's method
 *				b.Compare resultant error, take smaller one.
 *				c.  Find sign of error by cross product
 *			3. Find angle of current spot on road, compare to
 *			GPS heading angle to get heading error
 *
 *
 */


#define S_FUNCTION_NAME  lat_err_yaw4_switch_KT
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdio.h>

/*==================*
 * Global Variables *
 *==================*/
//Global variable (preserved between time steps) to count how many laps we've taken of the course
static int_T LapCount;
/*================*
 * Build checking *
 *================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 5);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, 6);
    
    ssSetNumSampleTimes(S, 1);
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE |
    SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

//Set lap count to zero
#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S)
{
    LapCount = 0;
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = 2*u
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    
    
    //4 outputs are error, segment, u, angle
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    
    //4 inputs are Segment number (from previous calculation, x position (East),
    //y position(west), and heading angle (CCW from North)
    InputRealPtrsType in = ssGetInputPortRealSignalPtrs(S,0);
    
    int NumSegs=10;
    double ax[10];
    double bx[10];
    double cx[10];
    double dx[10];
    double ex[10];
    double ay[10];
    double by[10];
    double cy[10];
    double dy[10];
    double ey[10];
    int bestseg, i, ij, seg, ii, imax;
    double error, x1, y0, y1, y2, B, C, temp, y10, y20,y21,xm,ym, tol, angle, rel_angle, bestxdot, bestydot, road_angle, lkswitch;
    double besterror,u, linex, liney, linexdot, lineydot, currenterror, bestu, bestx, besty, a, b, c, d, e, f, g, h, x0, x2,x,y;
    double xprime, yprime, xdoubleprime, ydoubleprime, kappa;
    besterror=100000; //Start with large best error to decide which segment is best;
    seg		=*in[0];
    x		=*in[1];
    y		=*in[2];
    angle	=*in[3];
    lkswitch=*in[4];
    
    
    Y[0]=0;        //Output to Y
    Y[1]=-1;            //Output to Y
    Y[2]=-1;             //Output to Y
    Y[3]=0;
    Y[4]=-1;
    Y[5] = -1;
    if (lkswitch) {
        //Here goes the segment data from the fit
        //Can be formatted using print params4.m
        
        //Rental-car friendly map
        //     ax[0]=0.965022;bx[0]=-6.216020;cx[0]=11.661356;dx[0]=31.707450;ex[0]=6.943861;
        //     ay[0]=6.711500;by[0]=1.047718;cy[0]=2.271544;dy[0]=-28.765014;ey[0]=-11.599691;
        //     ax[1]=-14.644036;bx[1]=9.942031;cx[1]=-1.196572;dx[1]=40.242191;ex[1]=45.061669;
        //     ay[1]=21.623830;by[1]=-48.027353;cy[1]=45.683700;dy[1]=5.767229;ey[1]=-30.333943;
        //     ax[2]=-13.975070;bx[2]=48.095279;cx[2]=-59.234698;dx[2]=9.098993;ex[2]=79.405282;
        //     ay[2]=31.675647;by[2]=-62.748124;cy[2]=31.344621;dy[2]=39.547890;ey[2]=-5.286537;
        //     ax[3]=9.189147;bx[3]=-20.623687;cx[3]=1.200720;dx[3]=-20.984845;ex[3]=63.389786;
        //     ay[3]=36.475574;by[3]=-89.582366;cy[3]=33.154133;dy[3]=40.695350;ey[3]=34.533497;
        //     ax[4]=-13.791003;bx[4]=31.188966;cx[4]=-5.535459;dx[4]=-43.697877;ex[4]=32.171122;
        //     ay[4]=-8.392900;by[4]=13.829184;cy[4]=-16.739519;dy[4]=-15.841185;ey[4]=55.276189;
        //     ax[5]=-15.563294;bx[5]=33.251902;cx[5]=5.285417;dx[5]=-16.365912;ex[5]=0.335748;
        //     ay[5]=-17.988539;by[5]=45.270715;cy[5]=-25.609366;dy[5]=-41.404270;ey[5]=28.131769;
        
        //Original Parking Garage Map
        //     ax[0]=14.361393;bx[0]=-35.134637;cx[0]=28.260492;dx[0]=35.271386;ex[0]=12.542361;
        //     ay[0]=-5.182453;by[0]=6.725517;cy[0]=24.935130;dy[0]=-33.953580;ey[0]=-19.012107;
        //     ax[1]=29.986484;bx[1]=-57.099820;cx[1]=9.024939;dx[1]=43.834031;ex[1]=55.300996;
        //     ay[1]=-5.929637;by[1]=9.174837;cy[1]=14.016962;dy[1]=15.363419;ey[1]=-26.487493;
        //     ax[2]=26.637497;bx[2]=-64.142387;cx[2]=17.644381;dx[2]=10.530384;ex[2]=81.046629;
        //     ay[2]=6.430459;by[2]=-21.675235;cy[2]=5.963650;dy[2]=47.203306;ey[2]=6.138088;
        //     ax[3]=-9.099900;bx[3]=21.015003;cx[3]=-14.957799;dx[3]=-40.058028;ex[3]=71.716504;
        //     ay[3]=-15.672694;by[3]=26.024132;cy[3]=-20.479300;dy[3]=19.826738;ey[3]=44.060269;
        //     ax[4]=-11.018148;bx[4]=31.391877;cx[4]=-6.512190;dx[4]=-43.328216;ex[4]=28.615780;
        //     ay[4]=-0.243679;by[4]=12.307154;cy[4]=-36.443071;dy[4]=-5.750243;ey[4]=53.759145;
        //     ax[5]=4.147051;bx[5]=-6.058788;cx[5]=21.554553;dx[5]=-6.249557;ex[5]=-0.850897;
        //     ay[5]=7.607692;by[5]=-6.575779;cy[5]=-0.983685;dy[5]=-42.689641;ey[5]=23.629305;
        
        //10 segment rental car-friendly map
//         ax[0]=1.062120;bx[0]=-3.878133;cx[0]=7.440688;dx[0]=17.158444;ex[0]=7.239630;
//         ay[0]=-1.982689;by[0]=3.289089;cy[0]=3.134774;dy[0]=-20.334343;ey[0]=-11.401996;
//         ax[1]=-1.507674;bx[1]=0.351016;cx[1]=2.179006;dx[1]=24.653898;ex[1]=29.022748;
//         ay[1]=-11.602387;by[1]=23.482398;cy[1]=1.105911;dy[1]=-12.128281;ey[1]=-27.295164;
//         ax[2]=-5.978846;bx[2]=11.383361;cx[2]=-5.813988;dx[2]=24.034265;ex[2]=54.698995;
//         ay[2]=5.489634;by[2]=-7.802515;cy[2]=1.938784;dy[2]=14.121187;ey[2]=-26.437523;
//         ax[3]=20.419453;bx[3]=-35.260000;cx[3]=-7.536984;dx[3]=22.640986;ex[3]=78.323787;
//         ay[3]=-1.503428;by[3]=-3.325418;cy[3]=11.469040;dy[3]=16.549744;ey[3]=-12.690433;
//         ax[4]=4.251782;bx[4]=-11.569617;cx[4]=9.199736;dx[4]=-16.535169;ex[4]=78.587242;
//         ay[4]=-7.204861;by[4]=14.242376;cy[4]=-7.527784;dy[4]=23.497858;ey[4]=10.499505;
//         ax[5]=-8.171640;bx[5]=10.176483;cx[5]=0.001578;dx[5]=-15.837419;ex[5]=63.933974;
//         ay[5]=-15.120338;by[5]=24.288356;cy[5]=-8.029820;dy[5]=22.349975;ey[5]=33.507094;
//         ax[6]=-6.079490;bx[6]=18.832092;cx[6]=-18.498814;dx[6]=-17.991375;ex[6]=50.102975;
//         ay[6]=9.838220;by[6]=-7.321118;cy[6]=-25.886780;dy[6]=18.674050;ey[6]=56.995266;
//         ax[7]=7.325502;bx[7]=-8.906274;cx[7]=1.520522;dx[7]=-22.810688;ex[7]=26.365388;
//         ay[7]=2.990532;by[7]=-13.010178;cy[7]=11.179190;dy[7]=-15.709981;ey[7]=52.299639;
//         ax[8]=11.183053;bx[8]=-21.060549;cx[8]=18.754708;dx[8]=-17.186461;ex[8]=3.494449;
//         ay[8]=-0.640573;by[8]=5.399463;cy[8]=-9.908149;dy[8]=-20.420004;ey[8]=37.749203;
//         ax[9]=7.413572;bx[9]=-19.904041;cx[9]=22.671380;dx[9]=1.873520;ex[9]=-4.814800;
//         ay[9]=-0.342142;by[9]=0.913607;cy[9]=2.446803;dy[9]=-26.600205;ey[9]=12.179940;
        
        //Shoreline oval
//         ax[0]=-0.258125;bx[0]=0.365647;cx[0]=0.148529;dx[0]=13.015383;ex[0]=9381.181027;
//         ay[0]=-1.548959;by[0]=3.504973;cy[0]=-2.231278;dy[0]=6.808188;ey[0]=-444.099367;
//         ax[1]=-1.327840;bx[1]=1.348404;cx[1]=-0.303277;dx[1]=13.376885;ex[1]=9394.452462;
//         ay[1]=0.425162;by[1]=1.035180;cy[1]=-1.010113;dy[1]=6.664714;ey[1]=-437.566443;
//         ax[2]=-1.208933;bx[2]=0.940307;cx[2]=-4.225107;dx[2]=11.504182;ex[2]=9407.546634;
//         ay[2]=0.121818;by[2]=-1.449951;cy[2]=4.646397;dy[2]=9.450674;ey[2]=-430.451501;
//         ax[3]=1.042806;bx[3]=-0.359397;cx[3]=-8.657783;dx[3]=1.039157;ex[3]=9414.557083;
//         ay[3]=1.352923;by[3]=-5.093368;cy[3]=1.027453;dy[3]=14.880887;ey[3]=-417.682563;
//         ax[4]=-1.092178;bx[4]=3.791576;cx[4]=-3.479136;dx[4]=-13.183374;ex[4]=9407.621867;
//         ay[4]=2.507816;by[4]=-3.882620;cy[4]=-6.135114;dy[4]=7.067380;ey[4]=-405.514668;
//         ax[5]=0.474794;bx[5]=-1.355591;cx[5]=1.342522;dx[5]=-13.135631;ex[5]=9393.658755;
//         ay[5]=-0.719441;by[5]=2.376879;cy[5]=-2.736075;dy[5]=-6.819441;ey[5]=-405.957205;
//         ax[6]=0.455298;bx[6]=-0.388490;cx[6]=0.124510;dx[6]=-12.618186;ex[6]=9380.984848;
//         ay[6]=-0.291123;by[6]=-0.075523;cy[6]=0.077918;dy[6]=-8.038717;ey[6]=-413.855283;
//         ax[7]=0.695778;bx[7]=1.167957;cx[7]=1.690829;dx[7]=-11.713444;ex[7]=9368.557980;
//         ay[7]=1.180046;by[7]=-2.129866;cy[7]=-1.895392;dy[7]=-9.273944;ey[7]=-422.182729;
//         ax[8]=-1.202751;bx[8]=0.625636;cx[8]=9.369366;dx[8]=-2.044804;ex[8]=9360.399100;
//         ay[8]=0.082869;by[8]=3.602163;cy[8]=-1.204713;dy[8]=-14.734141;ey[8]=-434.301885;
//         ax[9]=2.461374;bx[9]=-6.216495;cx[9]=4.029769;dx[9]=13.759832;ex[9]=9367.146546;
//         ay[9]=-2.473037;by[9]=0.835983;cy[9]=10.098993;dy[9]=-6.005600;ey[9]=-446.555706;
        
        //Shoreline map with long straightaways
     ax[0]=-11.094698;bx[0]=22.513590;cx[0]=-7.868577;dx[0]=16.690532;ex[0]=9379.447575;
     ay[0]=-6.458573;by[0]=15.007994;cy[0]=-10.564620;dy[0]=13.096709;ey[0]=-444.822133;
     ax[1]=-5.226116;bx[1]=11.657428;cx[1]=-6.895991;dx[1]=24.115359;ex[1]=9399.688423;
     ay[1]=-0.749696;by[1]=3.217880;cy[1]=-4.292074;dy[1]=11.157160;ey[1]=-433.740623;
     ax[2]=-3.978112;bx[2]=7.331950;cx[2]=-3.280405;dx[2]=24.391195;ex[2]=9423.339103;
     ay[2]=6.898596;by[2]=-9.638887;cy[2]=0.863387;dy[2]=9.227865;ey[2]=-424.407354;
     ax[3]=0.510141;bx[3]=-8.875659;cx[3]=-5.153226;dx[3]=23.913787;ex[3]=9447.803731;
     ay[3]=-5.739987;by[3]=3.255411;cy[3]=13.338300;dy[3]=9.632360;ey[3]=-417.056394;
     ax[4]=-7.264429;bx[4]=24.408332;cx[4]=-28.719357;dx[4]=-10.979078;ex[4]=9458.198774;
     ay[4]=8.721549;by[4]=-14.634753;cy[4]=-11.335391;dy[4]=23.115243;ey[4]=-396.570311;
     ax[5]=0.382951;bx[5]=-1.010752;cx[5]=0.919067;dx[5]=-24.250512;ex[5]=9435.644242;
     ay[5]=-0.738924;by[5]=2.673812;cy[5]=-2.910354;dy[5]=-8.573600;ey[5]=-390.703662;
     ax[6]=0.790663;bx[6]=-1.115673;cx[6]=0.184517;dx[6]=-23.912830;ex[6]=9411.684996;
     ay[6]=0.574814;by[6]=-1.246605;cy[6]=0.677536;dy[6]=-9.328569;ey[6]=-400.252729;
     ax[7]=3.952688;bx[7]=-4.790905;cx[7]=1.581476;dx[7]=-23.728163;ex[7]=9387.631673;
     ay[7]=-0.480156;by[7]=-1.927348;cy[7]=0.386606;dy[7]=-9.414057;ey[7]=-409.575553;
     ax[8]=4.125152;bx[8]=-2.112111;cx[8]=10.924891;dx[8]=-19.127173;ex[8]=9364.646769;
     ay[8]=6.029385;by[8]=-4.596956;cy[8]=-8.276376;dy[8]=-16.343514;ey[8]=-421.010509;
     ax[9]=8.833621;bx[9]=-30.069924;cx[9]=29.339469;dx[9]=12.886882;ex[9]=9358.457527;
     ay[9]=-16.060930;by[9]=23.897298;cy[9]=14.109064;dy[9]=-22.569595;ey[9]=-444.197970;
    
    //Shoreline map with even longer straightaways, gentler curvature    
//     ax[0]=-3.076134;bx[0]=4.936290;cx[0]=1.207170;dx[0]=22.951783;ex[0]=9415.310132;
//     ay[0]=-7.011183;by[0]=12.823802;cy[0]=-5.004467;dy[0]=20.916118;ey[0]=-444.456101;
//     ax[1]=-10.003243;bx[1]=5.328951;cx[1]=-2.440762;dx[1]=27.870460;ex[1]=9441.329242;
//     ay[1]=-9.263297;by[1]=22.007711;cy[1]=-8.600158;dy[1]=21.333859;ey[1]=-422.731831;
//     ax[2]=-8.112933;bx[2]=30.758879;cx[2]=-46.473369;dx[2]=-1.037184;ex[2]=9462.084647;
//     ay[2]=10.703359;by[2]=-25.933277;cy[2]=1.843191;dy[2]=33.103486;ey[2]=-397.253716;
//     ax[3]=-2.261683;bx[3]=6.394891;cx[3]=-2.874329;dx[3]=-34.159016;ex[3]=9437.220040;
//     ay[3]=0.564297;by[3]=0.706420;cy[3]=-11.736487;dy[3]=1.803473;ey[3]=-377.536957;
//     ax[4]=-0.868594;bx[4]=0.719842;cx[4]=2.740249;dx[4]=-29.769731;ex[4]=9404.319904;
//     ay[4]=-1.111103;by[4]=3.651755;cy[4]=-6.231446;dy[4]=-17.293053;ey[4]=-386.199254;
//     ax[5]=-3.723287;bx[5]=5.735034;cx[5]=-0.311788;dx[5]=-25.604082;ex[5]=9377.141670;
//     ay[5]=1.594658;by[5]=-1.584216;cy[5]=-1.942802;dy[5]=-23.245095;ey[5]=-407.183102;
//     ax[6]=-2.450446;bx[6]=16.281428;cx[6]=-5.446408;dx[6]=-23.915704;ex[6]=9353.237547;
//     ay[6]=6.543559;by[6]=-13.545507;cy[6]=2.872499;dy[6]=-25.504714;ey[6]=-432.360556;
//     ax[7]=1.370658;bx[7]=-11.205501;cx[7]=28.695199;dx[7]=4.233979;ex[7]=9337.706416;
//     ay[7]=0.314141;by[7]=10.165068;cy[7]=1.497334;dy[7]=-34.221999;ey[7]=-461.994718;
//     ax[8]=8.522265;bx[8]=-16.081955;cx[8]=3.302644;dx[8]=33.490506;ex[8]=9360.800751;
//     ay[8]=6.058688;by[8]=-23.310157;cy[8]=33.877385;dy[8]=0.524437;ey[8]=-484.240175;
//     ax[9]=5.192374;bx[9]=-12.045814;cx[9]=6.190370;dx[9]=25.938990;ex[9]=9390.034212;
//     ay[9]=-1.519029;by[9]=1.270222;cy[9]=0.299041;dy[9]=22.583487;ey[9]=-467.089822;
        
        //35 m radius artificially generated map
//        ax[0]=-0.164773;bx[0]=-3.363169;cx[0]=1.332463;dx[0]=0.075733;ex[0]=9360.677671;
//        ay[0]=-1.168490;by[0]=1.473593;cy[0]=-0.148240;dy[0]=27.533738;ey[0]=-462.828422;
//        ax[1]=1.207953;bx[1]=-0.750344;cx[1]=-9.745680;dx[1]=-8.007938;ex[1]=9358.557926;
//        ay[1]=1.095571;by[1]=-4.259696;cy[1]=-2.738402;dy[1]=26.984077;ey[1]=-435.137821;
//        ax[2]=0.199722;bx[2]=2.295144;cx[2]=-4.748994;dx[2]=-24.918519;ex[2]=9341.261916;
//        ay[2]=1.701416;by[2]=-3.602594;cy[2]=-8.944067;dy[2]=13.110466;ey[2]=-414.056272;
//        ax[3]=-0.663341;bx[3]=3.527519;cx[3]=3.334768;dx[3]=-26.732189;ex[3]=9314.089268;
//        ay[3]=1.374876;by[3]=-0.991734;cy[3]=-9.543356;dy[3]=-8.779788;ey[3]=-411.791051;
//        ax[4]=-1.715392;bx[4]=-0.321655;cx[4]=9.937275;dx[4]=-12.133463;ex[4]=9293.556024;
//        ay[4]=-0.695921;by[4]=2.866755;cy[4]=-4.269302;dy[4]=-25.342199;ey[4]=-429.731054;
//        ax[5]=0.131047;bx[5]=3.438193;cx[5]=-1.320042;dx[5]=-0.085447;ex[5]=9289.322788;
//        ay[5]=0.615575;by[5]=-0.529889;cy[5]=0.155440;dy[5]=-28.064219;ey[5]=-457.171720;
//        ax[6]=-1.181143;bx[6]=0.674823;cx[6]=9.780820;dx[6]=8.113236;ex[6]=9291.486539;
//        ay[6]=-1.414427;by[6]=5.019106;cy[6]=2.259224;dy[6]=-26.880705;ey[6]=-484.994812;
//        ax[7]=-0.186211;bx[7]=-2.324572;cx[7]=4.718434;dx[7]=24.974776;ex[7]=9308.874276;
//        ay[7]=-1.702940;by[7]=3.683265;cy[7]=8.829982;dy[7]=-12.962645;ey[7]=-506.011614;
//        ax[8]=0.672881;bx[8]=-3.539597;cx[8]=-3.372545;dx[8]=26.693085;ex[8]=9336.056703;
//        ay[8]=-1.058392;by[8]=0.479955;cy[8]=9.662136;dy[8]=8.935353;ey[8]=-508.163952;
//        ax[9]=1.661702;bx[9]=0.438767;cx[9]=-9.954052;dx[9]=12.020727;ex[9]=9356.510527;
//        ay[9]=1.267797;by[9]=-4.168891;cy[9]=4.751650;dy[9]=25.465922;ey[9]=-490.144900;
               
        //40 m radius artificially generated map
//         ax[0]=0.984932;bx[0]=-6.058536;cx[0]=1.431983;dx[0]=0.100586;ex[0]=9365.777727;
//         ay[0]=-1.393431;by[0]=1.414930;cy[0]=-0.044751;dy[0]=30.631373;ey[0]=-466.092869;
//         ax[1]=0.931382;bx[1]=0.212375;cx[1]=-10.834030;dx[1]=-11.271326;ex[1]=9362.236693;
//         ay[1]=1.119576;by[1]=-4.197666;cy[1]=-4.160547;dy[1]=29.212938;ey[1]=-435.484748;
//         ax[2]=0.035191;bx[2]=2.790419;cx[2]=-4.608610;dx[2]=-28.576731;ex[2]=9341.275095;
//         ay[2]=1.801693;by[2]=-3.646435;cy[2]=-10.036091;dy[2]=12.777148;ey[2]=-413.510448;
//         ax[3]=-0.838113;bx[3]=3.931000;cx[3]=3.973789;dx[3]=-29.281932;ex[3]=9310.915364;
//         ay[3]=1.441075;by[3]=-0.957514;cy[3]=-10.165239;dy[3]=-11.027567;ey[3]=-412.614132;
//         ax[4]=-1.723475;bx[4]=-0.598211;cx[4]=10.738111;dx[4]=-12.893806;ex[4]=9288.700108;
//         ay[4]=-0.794429;by[4]=3.068238;cy[4]=-4.391328;dy[4]=-28.466285;ey[4]=-433.323376;
//         ax[5]=-1.027821;bx[5]=6.134438;cx[5]=-1.397372;dx[5]=-0.106117;ex[5]=9284.222727;
//         ay[5]=0.768605;by[5]=-0.344554;cy[5]=0.046813;dy[5]=-31.221942;ey[5]=-463.907180;
//         ax[6]=-0.914059;bx[6]=-0.260749;cx[6]=10.839012;dx[6]=11.391166;ex[6]=9287.825854;
//         ay[6]=-1.472026;by[6]=5.037879;cy[6]=3.624782;dy[6]=-29.087558;ey[6]=-494.658259;
//         ax[7]=-0.021161;bx[7]=-2.818675;cx[7]=4.572410;dx[7]=28.630707;ex[7]=9308.881224;
//         ay[7]=-1.802458;by[7]=3.734921;cy[7]=9.906267;dy[7]=-12.612457;ey[7]=-516.555180;
//         ax[8]=0.853921;bx[8]=-3.952264;cx[8]=-4.010579;dx[8]=29.234859;ex[8]=9339.244505;
//         ay[8]=-1.089887;by[8]=0.390439;cy[8]=10.296282;dy[8]=11.195009;ey[8]=-517.328907;
//         ax[9]=1.680073;bx[9]=0.698462;cx[9]=-10.743843;dx[9]=12.772594;ex[9]=9361.370442;
//         ay[9]=1.425748;by[9]=-4.509172;cy[9]=4.928278;dy[9]=28.599342;ey[9]=-496.537065;
        
        //End of parameters
        
        bestseg=seg;
        if ((fabs(x)<300000) & (fabs(y)<300000)){
            for (ij=seg;ij<seg+2;ij++)
            {
                i=ij;
                if (ij==NumSegs+1)    //Wraparound to first segment if on final segment
                {
                    i=1;
                }
                //Mueller's method for finding polynomial roots
                a=4*pow(ax[i-1],2)+4*pow(ay[i-1],2);
                b=7*ax[i-1]*bx[i-1]+7*ay[i-1]*by[i-1];
                c=6*ax[i-1]*cx[i-1]+3*pow(bx[i-1],2)+6*ay[i-1]*cy[i-1]+3*pow(by[i-1],2);
                d=5*ax[i-1]*dx[i-1]+5*bx[i-1]*cx[i-1]+5*ay[i-1]*dy[i-1]+5*by[i-1]*cy[i-1];
                e=4*bx[i-1]*dx[i-1]+2*pow(cx[i-1],2)+4*ax[i-1]*ex[i-1]-4*ax[i-1]*x+4*by[i-1]*dy[i-1]+2*pow(cy[i-1],2)+4*ay[i-1]*ey[i-1]-4*ay[i-1]*y;
                f=-3*bx[i-1]*x+3*cx[i-1]*dx[i-1]+3*bx[i-1]*ex[i-1]+-3*by[i-1]*y+3*cy[i-1]*dy[i-1]+3*by[i-1]*ey[i-1];
                g=-2*cx[i-1]*x+dx[i-1]*dx[i-1]+2*cx[i-1]*ex[i-1]+-2*cy[i-1]*y+dy[i-1]*dy[i-1]+2*cy[i-1]*ey[i-1];
                h=-dx[i-1]*x+dx[i-1]*ex[i-1]+-dy[i-1]*y+dy[i-1]*ey[i-1];
                
                x0=-0.01;
                x2=1.01;
                //tol=0.0000000001;
                tol=  0.0000000001;
                //imax=50;
                imax=30;
                y0 = a*pow(x0,7)+b*pow(x0,6)+c*pow(x0,5)+d*pow(x0,4)+e*pow(x0,3)+f*pow(x0,2)+g*x0+h;
                y2 = a*pow(x2,7)+b*pow(x2,6)+c*pow(x2,5)+d*pow(x2,4)+e*pow(x2,3)+f*pow(x2,2)+g*x2+h;
                x1 = 0.5 * (x2 + x0);
                ii=0;
                while ((fabs(x1-x0) > tol) & (ii<imax))
                {
                    ii++;
                    x1 = 0.5 * (x2 + x0);
                    y1 = a*pow(x1,7)+b*pow(x1,6)+c*pow(x1,5)+d*pow(x1,4)+e*pow(x1,3)+f*pow(x1,2)+g*x1+h;
                    if (y1 * y0 > 0.0)
                    {
                        temp = x0;
                        x0 = x2;
                        x2 = temp;
                        temp = y0;
                        y0 = y2;
                        y2 = temp;
                    }
                    y10 = y1 - y0;
                    y21 = y2 - y1;
                    y20 = y2 - y0;
                    if (y2 * y20 <2.0 * y1 * y10)
                    {
                        x2 = x1;
                        y2 = y1;
                    }
                    else
                    {
                        B = (x1 - x0) / y10;
                        C = (y10 -y21) / (y21 * y20);
                        xm = x0 - B * y0 * (1.0-C * y1);
                        ym = a*pow(xm,7)+b*pow(xm,6)+c*pow(xm,5)+d*pow(xm,4)+e*pow(xm,3)+f*pow(xm,2)+g*xm+h;
                        if (ym * y0 < 0.0)
                        {
                            x2 = xm;
                            y2 = ym;
                        }
                        else
                        {
                            x0 = xm;
                            y0 = ym;
                            x2 = x1;
                            y2 = y1;
                        }
                    }
                }
                u=x1;
                
                linex=ax[i-1]*pow(u,4)+bx[i-1]*pow(u,3)+cx[i-1]*pow(u,2)+dx[i-1]*u+ex[i-1];	//  x position on line
                liney=ay[i-1]*pow(u,4)+by[i-1]*pow(u,3)+cy[i-1]*pow(u,2)+dy[i-1]*u+ey[i-1];	//  y position on line
                linexdot=4*ax[i-1]*pow(u,3)+3*bx[i-1]*pow(u,2)+2*cx[i-1]*u+dx[i-1];			//  dx/du
                lineydot=4*ay[i-1]*pow(u,3)+3*by[i-1]*pow(u,2)+2*cy[i-1]*u+dy[i-1];			//  dy/du
                currenterror=sqrt(pow((x-linex),2)+pow(y-liney,2));			//  mag of error is sqrt-sum-squares
                if (((x-linex)*lineydot-(y-liney)*linexdot)>0)				// check which side of the road is on
                {															// add neg sign if on right
                    currenterror=-currenterror;
                }
                if ((fabs(currenterror)<fabs(besterror)) & (u>-0.01 & u<1.01)) // if this is the better of the
                {															  // current and next segments
                    besterror=currenterror;
                    bestseg=i;
                    bestu=u;
                    bestx=linex;
                    besty=liney;
                    bestxdot=linexdot;
                    bestydot=lineydot;
                }
            }
        }
        else{
            besterror=0;
            bestseg=seg;
            bestu=0;
        }
        if (fabs(besterror)>1000){
            besterror=0;
            bestseg=seg;
        }
        
        //Assumes incoming angle is measured from North/Y, with
        //Counter-Clockwise Positive
        road_angle=atan2(bestxdot,bestydot);
        rel_angle=(angle+road_angle);
        while (rel_angle>3.1415)
        {
            rel_angle=rel_angle-2*3.1415;
        }
        while(rel_angle<-3.1415)
        {
            rel_angle=rel_angle+2*3.1415;
        }
        
        
        Y[0]=besterror;        //Output to Y
        Y[1]=bestseg;            //Output to Y
        Y[2]=bestu;             //Output to Y
        Y[3]=rel_angle;
        Y[4] = LapCount;
        
        //Compute necessary derivatives of x and y
        xprime = 4*ax[bestseg-1]*pow(bestu,3)+3*bx[bestseg-1]*pow(bestu,2) + 2*cx[bestseg-1]*bestu + dx[bestseg-1];
        yprime = 4*ay[bestseg-1]*pow(bestu,3)+3*by[bestseg-1]*pow(bestu,2) + 2*cy[bestseg-1]*bestu + dy[bestseg-1];
        xdoubleprime = 12*ax[bestseg-1]*pow(bestu,2) + 6*bx[bestseg-1]*bestu + 2*cx[bestseg-1];
        ydoubleprime = 12*ay[bestseg-1]*pow(bestu,2) + 6*by[bestseg-1]*bestu + 2*cy[bestseg-1];

        //...then compute the curvature
        kappa = (xprime*ydoubleprime - yprime*xdoubleprime)/pow(pow(xprime,2) + pow(yprime,2), 1.5);
        Y[5] = kappa;
        
        //If we have transitioned from the final segment back to the first
        //segment, increment the lap counter
        if (bestseg == 1 && seg == NumSegs) { 
            LapCount++;
        }
        
        
    }
    else{
        Y[0]=0;        //Output to Y
        Y[1]=0;            //Output to Y
        Y[2]=0;             //Output to Y
        Y[3]=0;
        Y[4]=0;
        Y[5] = 0;
    }
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
