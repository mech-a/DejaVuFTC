package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.dependencies.Robot;

@Autonomous(name = "Crater Side", group = "Auton")
public class SimpleCraterSide extends LinearOpMode {

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

            //TODO: function for dropping from lander

            //if we drop tilted we should probably align

            //read samples and align
            sleep(500);
            samplex = r.goldPos();
            if (samplex > DETECTOR_CENTER + DETECTOR_CENTER_THRESHOLD) {
                //the sample is to the right
                r.rotate("cw", ROTATE_SPEED, SAMPLE_ANGLE);
                samplePos = "right";
            }
            else if (samplex < DETECTOR_CENTER - DETECTOR_CENTER_THRESHOLD) {
                //the sample is to the left
                r.rotate("ccw", ROTATE_SPEED, SAMPLE_ANGLE);
                samplePos = "left";
            }
            else {
                //the sample is in the center
                samplePos = "center";
            }

            //knock off sample
            //for simplicity and fast coding, this currently goes to knock out the sample and just comes back
            sleep(600);
            if (samplePos == "center") {
                //forward and back about 35 inches
                r.translate(SAMPLE_DISTANCE_CENTER, TRANSLATE_SPEED);
                sleep(1000);
                r.translate(-SAMPLE_DISTANCE_CENTER, TRANSLATE_SPEED);
            }
            else {
                //forward and back about 42 inches
                r.translate(SAMPLE_DISTANCE_OTHER, TRANSLATE_SPEED);
                sleep(1000);
                r.translate(-SAMPLE_DISTANCE_OTHER, TRANSLATE_SPEED);
            }

            //the robot has returned to the starting position
            //movements towards depot
            sleep(600);
            r.rotate("ccw", ROTATE_SPEED, 45);
            sleep(600);
            r.translate(42, TRANSLATE_SPEED);
            sleep(600);
            r.rotate("cw", ROTATE_SPEED, 45);
            sleep(600);
            r.translate(48, TRANSLATE_SPEED);
            sleep(600);

            //TODO: function to drop marker

            //go to team side crater
            r.translate(-96, TRANSLATE_SPEED);
        }
    }
}
