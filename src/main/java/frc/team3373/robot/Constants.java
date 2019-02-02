/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.robot;

/**
 * Add your docs here.
 */
public class Constants {
    public static double[][] distanceTable1 = { //Data for sensor 1
        {0.42, 31},
        {0.44, 29},
        {0.46, 27},
        {0.48, 25},
        {0.53, 23},
        {0.56, 21},
        {0.60, 19},
        {0.65, 17},
        {0.74, 15},
        {0.79, 14},
        {0.83, 13},
        {0.89, 12},
        {0.94, 11},
        {1.04, 10},
        {1.16, 9},
        {1.29, 8},
        {1.41, 7},
        {1.63, 6},
        {1.90, 5},
        {2.40, 4},
        {3.00, 3}
    };

    public static double distanceSensora1 = 31.1534; //Constants for lineFit
    public static double distanceSensorb1 = 3.01684;
    public static double distanceSensorc1 = 0.0477401;
    public static double distanceSensord1 = 2.97265;
    public static double distanceSensore1 = 4.14756;
    public static double distanceSensorf1 = -0.00588378;

    public static double[][] distanceTable2 = { //Data for sensor 2
        {0.43, 31},
        {0.45, 29},
        {0.47, 27},
        {0.50, 25},
        {0.54, 23},
        {0.58, 21},
        {0.62, 19},
        {0.68, 17},
        {0.76, 15},
        {0.79, 14},
        {0.85, 13},
        {0.91, 12},
        {1.00, 11},
        {1.08, 10},
        {1.18, 9},
        {1.32, 8},
        {1.50, 7},
        {1.75, 6},
        {2.07, 5},
        {2.51, 4},
        {3.14, 3}
    };

    public static double distanceSensora2 = 31.1277;
    public static double distanceSensorb2 = 2.03942;
    public static double distanceSensorc2 = 1.285;
    public static double distanceSensord2 = 2.97952;
    public static double distanceSensore2 = 3.08719;
    public static double distanceSensorf2 = 0.00183152;

    public static double[][] distanceTable3 = {
        {0.39, 31},
        {0.42, 29},
        {0.44, 27},
        {0.48, 25},
        {0.51, 23},
        {0.57, 21},
        {0.57, 19},
        {0.64, 17},
        {0.70, 15},
        {0.75, 14},
        {0.79, 13},
        {0.85, 12},
        {0.93, 11},
        {1.00, 10},
        {1.10, 9},
        {1.24, 8},
        {1.35, 7},
        {1.54, 6},
        {1.73, 5},
        {2.14, 4},
        {2.76, 3}
    };

    public static double distanceSensora3 = 20.0191;
    public static double distanceSensorb3 = -1.25134;
    public static double distanceSensorc3 = 0.434041;
    public static double distanceSensord3 = 4.29208;
    public static double distanceSensore3 = -0.00667298;
    public static double distanceSensorf3 = -2.24084;



    public static double FLP = 5;//close loop error =1
    public static double FLI = 0.001;
    public static double FLD = 25;

    public static double FRP = 5;//close loop error =1
    public static double FRI = 0.0015;
    public static double FRD = 25;

    public static double BLP = 6;//close loop error =1
    public static double BLI = 0.0035;
    public static double BLD = 25;

    public static double BRP = 5;//close loop error =1
    public static double BRI = 0.0015;
    public static double BRD = 15;
}
