package org.firstinspires.ftc;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Robot;


@Autonomous(name = "Depot Side", group = "Auton")
public class DepotSideAuton extends LinearOpMode {

    Robot r = new Robot(this);


    //constants that will probably be moved to the Constants class

    public static final double DETECTOR_CENTER = 200;
    public static final double DETECTOR_CENTER_THRESHOLD = 15;
    public static final double SAMPLE_ANGLE = 25;
    public static final double SAMPLE_DISTANCE_CENTER = 35;
    public static final double SAMPLE_DISTANCE_OTHER = 42;
    public static final double TRANSLATE_SPEED = 0.25;
    public static final double ROTATE_SPEED = 0.25;
    double speed = 0.3;

    Enums.GoldPosition Pos = Enums.GoldPosition.MIDDLE;

    double samplex = 0;
    String samplePos = "left";

    @Override public void runOpMode() {
        r.start(hardwareMap, telemetry);
        r.init();
        r.cvInit();
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

        r.positionDrive(2, -40, 0.2);

        sleep(100);
        r.servoMotors[1].setPosition(0.35);
        sleep(100);




        //r.positionDrive(0,840,0.3 );

        r.armMotors[0].setPower(0.5);
        while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 770) {}
        r.armMotors[0].setPower(0);


        sleep(100);

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

          r.strafe(4, 0.2);
          sleep(100);
          r.translate(-3, speed);
          sleep(100);
          r.strafe(-8, 0.2);
          sleep(100);


          // detect mineral
          Pos = r.getGoldPosition();

          sleep(30000);

          if (Pos == Enums.GoldPosition.RIGHT) {
              telemetry.addData("side:", "right");
              samplePos = "right";
          } else if (Pos == Enums.GoldPosition.MIDDLE) {
              telemetry.addData("side:", "middle");
              samplePos = "center";
          } else if (Pos == Enums.GoldPosition.LEFT) {
              telemetry.addData("side:", "left");
              samplePos = "left";
          }
          telemetry.update();

          sleep(100);
          r.translate(-6, speed);
          sleep(100);




//        r.rotate("cw",0.1,18);
//        //TODO translate 4
//        r.translate(4,-0.15);
//        r.rotate("ccw",0.1,0);

//        r.rotate("ccw",0.05,31);
//        r.translate(28.5,-0.05);
//        r.rotate("cw",0.05,20);

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


//        r.rotate("ccw", 0.1, 35);
//        r.translate(30.8, -0.1);
//        r.rotate("cw",0.05,35);
//        r.translate(39.3,-0.05);
//        r.servoMotors[0].setPosition(0);


        //TODO check if last motor hits pos or first motor hits position



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


        if(Pos == Enums.GoldPosition.RIGHT) {
            r.strafe(-15, 0.2);
            sleep(100);
            r.translate(-20, speed);
            sleep(100);
            r.rotate("ccw", 0.15, 22);
            sleep(100);
            r.translate(-23.3,speed);
        } else if (Pos == Enums.GoldPosition.MIDDLE) {
            r.strafe(4, 0.4);
            r.translate(-43, speed);
        } else {
            r.strafe(16, 0.2);
            sleep(100);
            r.translate(-20, speed);
            sleep(100);
            r.rotate("cw", 0.15, 20);
            sleep(100);
            r.translate(-25.3,speed);
        }



            //placeholder for dropping the team marker
            sleep(100);
        r.rotate("cw",0.15,44);
            sleep(100);
//        r.servoMotors[0].setPosition(0);
        r.positionDrive(2, -220, 0.2);
        sleep(500);
        r.positionDrive(2, 220, 0.2);

        //rotate to 45deg instead of .5?
            sleep(100);
            r.strafe(12, 0.2);
            sleep(100);
            r.strafe(-3, 0.2);
            //r.armMotors[0].setPower(0.5);

            sleep(100);
            r.translate(75,speed + 0.1);


            //.addData("xpos",detector.getScreenPosition().x);



            telemetry.update();








    }
}
