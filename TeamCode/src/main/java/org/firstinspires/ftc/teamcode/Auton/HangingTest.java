package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dependencies.Robot;

@Autonomous(name = "Hanging Test", group = "Auton")
public class HangingTest extends LinearOpMode {

    Robot r = new Robot(this);


    //constants that will probably be moved to the Constants class
    public static final double DETECTOR_CENTER = 200;
    public static final double DETECTOR_CENTER_THRESHOLD = 15;
    public static final double SAMPLE_ANGLE = 25;
    public static final double SAMPLE_DISTANCE_CENTER = 35;
    public static final double SAMPLE_DISTANCE_OTHER = 42;
    public static final double TRANSLATE_SPEED = 0.25;
    public static final double ROTATE_SPEED = 0.25;

    double samplex = 0;
    String samplePos = "center";

    @Override public void runOpMode() {
        r.start(hardwareMap, telemetry);
        r.init();

        waitForStart();

        while(opModeIsActive()) {
            //telemetry("counts of motor", driveMotors[])


        }
    }
}
