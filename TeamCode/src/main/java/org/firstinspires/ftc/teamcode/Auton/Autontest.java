package org.firstinspires.ftc.teamcode.Auton;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DogeCVTesting.CustomGoldDetector;
import org.firstinspires.ftc.teamcode.dependencies.Robot;

@Autonomous(name = "Auton Test", group = "Auton")
public class Autontest extends LinearOpMode {

    Robot r = new Robot(this);


    //constants that will probably be moved to the Constants class
    private CustomGoldDetector detector;
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
        r.driveMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.driveMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        detector = new CustomGoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.enable();

        waitForStart();


            //r.driveMotors[0].setTargetPosition(540);
            //.driveMotors[0].setPower(0.1);
            //while(r.driveMotors[0].isBusy()){

            //}
            //r.positionDrive(0,854,0.5);
            if(detector.isFound()){
                r.translate(16,0.1);
            }
            else {
                r.rotate("clockwise", 0.1, 10);
            }
            if(detector.isFound()){
                r.translate(18,0.1);
            }
            else{
                r.rotate("ccw",0.1,10);
                r.translate(18,0.1);
            }





    }
}
