package org.firstinspires.ftc.teamcode.Auton;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DogeCVTesting.CustomGoldDetector;
import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Robot;


@Autonomous(name = "Depot Side", group = "Auton")
public class DepotSideAuton extends LinearOpMode {

    Robot r = new Robot(this, Enums.OpModeType.AUTON);


    //constants that will probably be moved to the Constants class
    private CustomGoldDetector detector;
    private double speed = 0.15;
    private int time = 50;
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
        //r.driveMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //r.driveMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        detector = new CustomGoldDetector();
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//        detector.useDefaults();
//
//        detector.enable();

        telemetry.addData("Stat:" ,"Initialized");
        telemetry.update();

        waitForStart();

        r.armMotors[0].setPower(-0.25);

        sleep(100);
        r.servoMotors[1].setPosition(0.35);
        sleep(100);

        //r.positionDrive(0,840,0.3 );

        r.armMotors[0].setPower(0.5);
        while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 840) {}
        r.armMotors[0].setPower(0);


        sleep(100);

        Enum goldPos = r.getGoldPosition();

        r.translate(Enums.Direction.RIGHT, 2, 0.25);

        r.translate(4,-0.15);

        //TODO check if last motor hits pos or first motor hits position


        if (goldPos==Enums.GoldPosition.MIDDLE) {
            telemetry.addData("Position:", "Center");
            telemetry.update();
            //make this less before running
            r.translate(Enums.Direction.LEFT, 2, speed);
            r.translate(Enums.Direction.BACK,54, speed);
            }
        else if (goldPos==Enums.GoldPosition.RIGHT) {
            r.translate(Enums.Direction.BACK, 13,speed);
            sleep(time);
            r.translate(Enums.Direction.LEFT, 14, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,41, speed);
            sleep(time);
            r.translate(Enums.Direction.RIGHT, 12, speed);
                sleep(time);
        }else{
            r.translate(Enums.Direction.BACK, 13,speed);
            sleep(time);
            r.translate(Enums.Direction.RIGHT, 12, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,41, speed);
            sleep(time);
            r.translate(Enums.Direction.LEFT, 12, speed);
            sleep(time);
        }

        //placeholder for dropping the team marker
        r.positionDrive(2, 800,0.5);
        sleep(100);
        r.positionDrive(2, 0, 0.5);

        //rotate to 45deg instead of .5?
        r.rotate("cw",0.1,45);
        sleep(time);
        r.translate(Enums.Direction.RIGHT, 16, speed+0.1);
        sleep(time);
        r.translate(Enums.Direction.LEFT, 2, speed);
        sleep(time);
        r.translate(Enums.Direction.FWD,75,0.2);

        telemetry.update();
        detector.disable();
    }
}
