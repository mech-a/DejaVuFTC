package org.firstinspires.ftc.teamcode.called;



/**
 * Created by gbhat on 11/5/2017.
 */

public class RobotValues {
    //SRV L CLOSED IS 1, OPENED 0
    //SRV R CLOSED IS 0, OPENED 1
    //COUNTSPERINCH should be 252 now
    public static int COUNTS_PER_INCH = 126;

    //CPI for neverest 60


    //Static vars for movement in autons; used for both back autons

    public static double JEWEL_SERVO_DOWN = 0;
    public static double JEWEL_SERVO_UP = 1;


    //speed for conveyor
    public static double SPEED_FOR_CONVEYORS = 0.6;
    //counts to vuf + crypto = 36 * countsPerInch
    public static double SPEED_TO_VUFORIA = 0.2;
    public static int COUNTS_TO_VUFORIA = 12 * COUNTS_PER_INCH;

    public static double SPEED_TO_CRYPTO = 0.2;
    public static int COUNT_TO_CRYPTO = 24 * COUNTS_PER_INCH;

    public static double SPEED_TO_TURN = 0.2;
    public static double DEGREES_TO_TURN_FOR_CRYPTO = 90;

    //actual dist is 7.63 ; test and check; currently 7.5inches
    public static int COUNTS_BETWEEN_COLUMNS = 945;

    public static double SPEED_TO_PLACE_GLYPH = 0.2;
    public static int COUNTS_TO_PLACE_GLYPH = 6 * COUNTS_PER_INCH;

    public static int COUNTS_TO_VUFORIA_FRONT = 24 * COUNTS_PER_INCH;
    public static int COUNTS_TO_CRYPTO_FRONT = 12 * COUNTS_PER_INCH;

    //Static vars for detecting jewel colors
    public static final double SCALE_FACTOR = 255;
    //Do remember that for red, it ranges from 340 -> 360 -> 15 like a unit circle
        public static float BLUE_LOWER_LIMIT = 215;
        public static float BLUE_UPPER_LIMIT = 270;
        public static float RED_LOWER_LIMIT = 15;
        public static float RED_UPPER_LIMIT = 340;
}
