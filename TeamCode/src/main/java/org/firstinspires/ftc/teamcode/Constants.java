//TODO javadocs for constants file


package org.firstinspires.ftc.teamcode;

public class Constants {
    //keeping everything double so we don't accidentally int divide

    //Core motor specs:
    public static final double CORE_COUNTS_PER_REV = 288;
    public static final double HD_COUNTS_PER_REV = 560;
    public static final double WHEEL_DIAM = 4;

    //this is equal to your exact gear ratio
    public static final double GEAR_RATIO_DRIVE = 1;
    public static final int HD_COUNTS_PER_INCH = (int) (HD_COUNTS_PER_REV * GEAR_RATIO_DRIVE / (Math.PI * WHEEL_DIAM));

    //gear ratio for lifting arm should be 3:1, 3 revs = 1 turn so
    public static final double LIFT_GEAR_RATIO_MULTIPLIER = (1/3);
    //TODO ^ is a ghetto double


    //TODO make constants based off of robot



}
