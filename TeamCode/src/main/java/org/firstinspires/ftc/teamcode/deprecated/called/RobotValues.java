package org.firstinspires.ftc.teamcode.called;



/**
 * Created by gbhat on 11/5/2017.
 */

public class RobotValues {
    //SRV L CLOSED IS 1, OPENED 0
    //SRV R CLOSED IS 0, OPENED 1
    //COUNTSPERINCH should be 252 now, 126 previously w true holo
    //UPDATE: counts per inch is 45 due to bad gear ratios
    //public static int COUNTS_PER_INCH = 45;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NV40
    //new 1/2 gr
    static final double DRIVE_GEAR_REDUCTION = 0.5;
    //static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP



    // new gear ratio multiplier: 1 rotation of 2:1 is 1/2 wheel
    //1 rotation of 1:2 is 2 * wheel
    //new mult is 1/2

    static final double GRCHANGE_MULTIPLIER = 0.5;


    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final int     COUNTS_PER_INCH         = (int) ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));

    static final int robotDiameter = (int)(18 * Math.sqrt(2));

    public static int COUNTS_FOR_FULL_ROTATION = (int) (robotDiameter * 3.1415 * COUNTS_PER_INCH);

    //CPI for neverest 60


    //Static vars for movement in autons; used for both back autons

    public static double ARM_JEWEL_DOWN = 0.93;
    public static double ARM_JEWEL_UP = 0.3;
    public static double HITTER_JEWEL_MIDDLE = 0.5;
    //south points to back of robot, north front
    public static double HITTER_JEWEL_SOUTH = 0;
    public static double HITTER_JEWEL_NORTH = 1;


    //speed for conveyor
    public static double SPEED_FOR_CONVEYORS = 0.6;

    //counts per 6inch neverest 60
    // six inch not accurate atm
    public static int SIX_INCHES_NV60 = 5040;
    public static int SEVEN_INCHES_NV60 = 2940;

    public static int CPI_NV60 = 1680/4;
    public static int SIXPOINTFIVE_NV60 = (int)(CPI_NV60 * 6.5);

    //neverest 60 speed
    public static double NV60_SPEED = 0.65;

    //extruder servo speed
    public static double EXTRUDER_SPEED = 1.0;

    //counts to vuf + crypto = 36 * countsPerInch
    public static double SPEED_TO_VUFORIA = 0.4 * GRCHANGE_MULTIPLIER;
    public static int COUNTS_TO_VUFORIA = 10 * COUNTS_PER_INCH;

    public static double SPEED_TO_CRYPTO = 0.4 * GRCHANGE_MULTIPLIER;
    public static int COUNT_TO_CRYPTO = 26 * COUNTS_PER_INCH;

    public static double SPEED_TO_TURN = 0.2 * GRCHANGE_MULTIPLIER;
    public static double DEGREES_TO_TURN_FOR_CRYPTO = 90;

    //actual dist is 7.63 ; test and check; currently 7.5inches
    public static int COUNTS_BETWEEN_COLUMNS = (int)(7.63 * COUNTS_PER_INCH);
    public static int COUNTS_TO_GET_TO_EDGE_OF_CRYPTO = (int)(COUNTS_PER_INCH * (36 - 7.63 - 1));
    public static int COUNTS_FOR_CLOSE_VUF = COUNTS_TO_GET_TO_EDGE_OF_CRYPTO;
    public static int COUNTS_FOR_MID_VUF = COUNTS_TO_GET_TO_EDGE_OF_CRYPTO + (1 * COUNTS_BETWEEN_COLUMNS);
    public static int COUNTS_FOR_FAR_VUF = COUNTS_TO_GET_TO_EDGE_OF_CRYPTO + (2 * COUNTS_BETWEEN_COLUMNS);



    public static double SPEED_TO_PLACE_GLYPH = 0.3 * GRCHANGE_MULTIPLIER;
    public static int COUNTS_TO_PLACE_GLYPH = 8 * COUNTS_PER_INCH;

    public static double DEADZONE = 0.1;


    public static int COUNTS_TO_CRYPTO_FRONT = 14 * COUNTS_PER_INCH;

    public static int NEW_COUNTS_TO_CRYPTO_FRONT = 24* COUNTS_PER_INCH;

    public static int COUNTS_TO_GET_TO_MID_FRONT = (int)((7.63/2) * COUNTS_PER_INCH);

    //Static vars for detecting jewel colors
    public static final double SCALE_FACTOR = 255;
    //Do remember that for red, it ranges from 340 -> 360 -> 15 like a unit circle
        public static float BLUE_LOWER_LIMIT = 215;
        public static float BLUE_UPPER_LIMIT = 270;
        public static float RED_LOWER_LIMIT = 15;
        public static float RED_UPPER_LIMIT = 340;

    public static double INTAKE_CLAW_POWER = 0.5;
    public static double EXTRUDE_CLAW_POWER = -INTAKE_CLAW_POWER;
}
