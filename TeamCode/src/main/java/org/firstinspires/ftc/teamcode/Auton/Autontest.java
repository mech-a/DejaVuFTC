package org.firstinspires.ftc.teamcode.Auton;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DogeCVTesting.CustomGoldDetector;
import org.firstinspires.ftc.teamcode.dependencies.Robot;


@Autonomous(name = "Auton test 2", group = "Auton")
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
        //r.driveMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //r.driveMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        detector = new CustomGoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.enable();



        telemetry.addData("Stat:" ,"Initialized");
        telemetry.update();

        waitForStart();


            //r.driveMotors[0].setTargetPosition(540);
            //.driveMotors[0].setPower(0.1);
            //while(r.driveMotors[0].isBusy()){



        r.armMotors[0].setPower(-0.25);

        sleep(350);
        r.servoMotors[1].setPosition(0.35);
        sleep(350);




        r.positionDrive(0,840,0.2 );


            sleep(3000);

            //28 inches to go forward to center
            //26.5 inches to center mineral for angle measuring
            //31.5/2 inches from mineral
            //30.7 degrees to mineral
            //36 inches from mineral to depot
            //for left mineral turn 30.7 degrees ccw and move 30.8 inches then 66.4 degrees cw and move 39.3 inches
            //for right mineral turn 30.7 degreescw and move 30.8 inches then 66.4 degrees ccw and move 39.3 inches
            //for center mineral go straight 62.5 inches
            // and then move 45 degrees ccw and move 72 inches for crater


        //for left mineral
//        r.rotate("ccw",45, 0.05);
//        r.positionDrive(0,-800,0.5);
//        r.rotate("cw",0.05,0);
        //18-19 or 11.6 degrees cw to rotate out of the hook
        //drive 2 inches forward
        //8 or 10 inches forward
        //rotate 31 ccw degrees or 41
        //28.5 inches forward
        //rotate 21 degrees cw
        //34 inches forward
        r.rotate("cw",0.05,12);
        r.translate(2,-0.05);
        r.rotate("ccw",0.05,31);
        r.translate(28.5,-0.05);
        r.rotate("cw",0.05,20);

//

//        r.translate(4,-0.1);
//        r.rotate("ccw", 0.1, 27);
//        r.translate(30.8, -0.1);
//        r.rotate("cw",0.05,35);
//        r.translate(39.3,-0.05);
//        sleep(1000);
//        r.rotate("ccw",0.1,40);
//        r.translate(65,0.1);
//        r.translate(10,0.05);

        /*
        // for right mineral
        r.translate(4,-0.1);
        r.rotate("cw", 0.1, 35 );
        r.translate(30.8, -0.1);
        r.rotate("ccw",0.05,28);
        r.translate(39.3,-0.1);
        sleep(1000);
        r.rotate("ccw",0.05,45);
        r.translate(65,0.1);
        r.translate(10,0.05);
        */


            /*
            if (detector.getScreenPosition().x < 400 && detector.getScreenPosition().x > 200) {

                r.translate(62.5, -0.1);


            }
            else {
                r.translate(4,-0.1);
                r.rotate("ccw", 0.1, 30.7);
                sleep(500);
                if (detector.getScreenPosition().x < 400 && detector.getScreenPosition().x > 200) {
                    r.translate(30.8, -0.1);
                    r.rotate("cw",0.05,35);
                    r.translate(39.3,-0.05);
                }
                else{
                    r.rotate("cw", 0.05, 30.7);
                    r.translate(30.8, -0.1);
                    r.rotate("ccw",0.05,35);
                    r.translate(39.3,-0.05);

                }

            }
            //placeholder for dropping the team marker
            sleep(1000);
            r.rotate("ccw",0.1,40);
            r.translate(75,0.1);

            telemetry.addData("xpos",detector.getScreenPosition().x);



            telemetry.update();
            */

        detector.disable();





    }
}
