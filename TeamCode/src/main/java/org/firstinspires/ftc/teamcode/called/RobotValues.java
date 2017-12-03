package org.firstinspires.ftc.teamcode.called;



/**
 * Created by gbhat on 11/5/2017.
 */

public class RobotValues {
    //SRV L CLOSED IS 1, OPENED 0
    //SRV R CLOSED IS 0, OPENED 1
    //COUNTSPERINCH should be 252 now, 126 previously w true holo
    //UPDATE: counts per inch is 45 due to bad gear ratios
    public static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    public static final int     COUNTS_PER_INCH         = (int) ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)) ;

    //CPI for neverest 60
    public static final int COUNTS_PER_INCH_NV60 = 840;

    public static String vuforiaKey = "AboeStD/////AAAAGaA8AMxAS0isjCVUhD" +
            "5lHuRymY1yqEVbDu1/PRTIEg/9JzZxKpV/P" +
            "39rY/QC64WcjeCtnUDq0jj7yWEPkWZClL" +
            "RC2KVwsjQPUe/mjwl6y51KfIKgSulpN63f" +
            "EYOBdY5ZR4fNswicR46PElRn4NaKqkV6fr9cLS62V8O" +
            "a8ow88oUK3xga8OJkNYf+3VoIQ7dj/RxiKzQCBJRt" +
            "2ZbRIUimTFw4oTC5LJ/NXV2jSD+m7KnW7TCpC7n/7hRxyKR" +
            "mw+JKGoz5kJIfxhliqs1XD3MnD9KN5w6cEwEmg3uYUZ5Bx7bcuO" +
            "N/uEaqifBnmwpdI0Vjklr67kMVYb27z1NsC+OB7moGIPdjhKho6nhwLy9XyMPw";

    //Static vars for movement in autons; used for both back autons

    public static final double JEWEL_SERVO_DOWN = 0;
    public static final double JEWEL_SERVO_UP = 1;

    //speed for conveyor
    public static final double SPEED_FOR_CONVEYORS = 0.6;

    //counts per 6inch neverest 60
    public static final int SIX_INCHES_NV60 = 5040;

    //neverest 60 speed
    public static final double NV60_SPEED = 0.6;

    //extruder servo speed
    public static final double EXTRUDER_SPEED = 1.0;

    //counts to vuf + crypto = 36 * countsPerInch
    public static final double SPEED_TO_VUFORIA = 0.2;
    public static final int COUNTS_TO_VUFORIA = 12 * COUNTS_PER_INCH;

    public static final double SPEED_TO_CRYPTO = 0.2;
    public static final int COUNT_TO_CRYPTO = 24 * COUNTS_PER_INCH;

    public static final double SPEED_TO_TURN = 0.2;
    public static final double DEGREES_TO_TURN_FOR_CRYPTO = 90;

    //actual dist is 7.63 ; test and check; currently 7.5inches
    public static final int COUNTS_BETWEEN_COLUMNS = 945;

    public static final double SPEED_TO_PLACE_GLYPH = 0.2;
    public static final int COUNTS_TO_PLACE_GLYPH = 6 * COUNTS_PER_INCH;

    public static final int COUNTS_TO_VUFORIA_FRONT = 10 * COUNTS_PER_INCH;
    public static final int COUNTS_TO_CRYPTO_FRONT = 14 * COUNTS_PER_INCH;

    //Static vars for detecting jewel colors
    public static final double SCALE_FACTOR = 255;
    //Do remember that for red, it ranges from 340 -> 360 -> 15 like a unit circle
        public static float BLUE_LOWER_LIMIT = 215;
        public static float BLUE_UPPER_LIMIT = 270;
        public static float RED_LOWER_LIMIT = 15;
        public static float RED_UPPER_LIMIT = 340;
}
