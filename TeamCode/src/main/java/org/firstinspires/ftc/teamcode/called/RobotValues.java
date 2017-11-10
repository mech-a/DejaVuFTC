package org.firstinspires.ftc.teamcode.called;



/**
 * Created by gbhat on 11/5/2017.
 */

public class RobotValues {
    //SRV L CLOSED IS 1, OPENED 0
    //SRV R CLOSED IS 0, OPENED 1
    public static int COUNTS_PER_INCH = 126;

    //Static vars for movement in autons; used for both back autons

    public static double JEWEL_SERVO_DOWN = 0;
    public static double JEWEL_SERVO_UP = 1;

    //counts to vuf + crypto = 36 * countsPerInch
    public static double SPEED_TO_VUFORIA = 0;
    public static int COUNTS_TO_VUFORIA = 22 * COUNTS_PER_INCH;

    public static double SPEED_TO_CRYPTO = 0;
    public static int COUNT_TO_CRYPTO = 14 * COUNTS_PER_INCH;

    public static double SPEED_TO_TURN = 0;
    public static double DEGREES_TO_TURN_FOR_CRYPTO = 90;

    //actual dist is 7.63 ; test and check
    public static int DISTANCE_BETWEEN_COLUMNS= 7;

    public static double SPEED_TO_PLACE_GLYPH = 0;
    public static int COUNTS_TO_PLACE_GLYPH = 6 * COUNTS_PER_INCH;

    public static int COUNTS_TO_VUFORIA_FRONT = 24;

    //Static vars for detecting jewel colors
    public static final double SCALE_FACTOR = 255;
    //Do remember that for red, it ranges from 340 -> 360 -> 15 like a unit circle
        public static float BLUE_LOWER_LIMIT = 215;
        public static float BLUE_UPPER_LIMIT = 270;
        public static float RED_LOWER_LIMIT = 15;
        public static float RED_UPPER_LIMIT = 340;
}
