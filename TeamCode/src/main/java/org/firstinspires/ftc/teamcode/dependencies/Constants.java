//TODO javadocs for constants file


package org.firstinspires.ftc.teamcode.dependencies;

public class Constants {
    //keeping everything double so we don't accidentally int divide

    //DT Specs:
    public static final double HD_COUNTS_PER_REV = 560;
    //in inches
    public static final double WHEEL_DIAM = 4;

    //this is equal to your exact gear ratio
    public static final double DRIVE_GEAR_RATIO = 1;
    public static final int HD_COUNTS_PER_INCH =
            getCountsPerInch(DRIVE_GEAR_RATIO, HD_COUNTS_PER_REV, WHEEL_DIAM);





    //Arm Specs:
    public static final double CORE_COUNTS_PER_REV = 288;
    //gear ratio for lifting arm should be 3:1, 3 revs = 1 turn so
    public static final double LIFT_GEAR_RATIO = 3;
    //TODO get these vals
    public static final double LINKS_PER_INCH = 5;
    public static final double TEETH_ON_OUTPUT_GEAR = 25;
    public static final int LIFT_COUNTS_PER_INCH =
            getCountsPerInch(LIFT_GEAR_RATIO, CORE_COUNTS_PER_REV, LINKS_PER_INCH, TEETH_ON_OUTPUT_GEAR);

    public static final int COUNTS_TO_DROP = 854;

    public static final int TELESCOPING_GEAR_RATIO = 1;
    //TODO get diam
    public static final double PULLEY_DIAM = 2;
    public static final int TELESCOPING_COUNTS_PER_INCH =
            getCountsPerInch(TELESCOPING_GEAR_RATIO, CORE_COUNTS_PER_REV, PULLEY_DIAM);


    //TODO set
    public static final int TELESCOPING_MAX_POSITION = 111111111 * TELESCOPING_COUNTS_PER_INCH;


    //TODO get these other vals
    public static final double ROTATION_MAX_POSITION = 0;
    public static final double ROTATION_MIN_POSITION = 0;



    public static final int RIGHT_BOUND = 400;
    public static final int LEFT_BOUND = 200;




    private static int getCountsPerInch(double gearRatio, double countsRevolution, double diam) {
        return (int) (gearRatio * countsRevolution / (Math.PI * diam));
    }

    private static int getCountsPerInch(double gearRatio, double countsRevolution,
                                        double linksPerInch, double teethOnOutputGear) {
        return (int) (gearRatio * countsRevolution * linksPerInch / teethOnOutputGear);
    }

    //TODO make constants based off of robot



}
