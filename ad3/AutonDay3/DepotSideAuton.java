package org.firstinspires.ftc.teamcode.AutonDay3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



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
        while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 840) {}
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

          r.strafe(4, 0.1);
          sleep(100);
          r.translate(-5, 0.15);
          sleep(100);
          r.strafe(-7, 0.1);
          sleep(100);


          // detect mineral

          if (r.getGoldPosition() == Enums.GoldPosition.RIGHT) {
              telemetry.addData("side:", "right");
              samplePos = "right";
          } else if (r.getGoldPosition() == Enums.GoldPosition.MIDDLE) {
              telemetry.addData("side:", "middle");
              samplePos = "center";
          } else if (r.getGoldPosition() == Enums.GoldPosition.LEFT) {
              telemetry.addData("side:", "left");
              samplePos = "left";
          }
          sleep(300);
          r.translate(-6, 0.15);
          sleep(500);




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


        if(samplePos == "right") {
            r.strafe(-12, 0.1);
            sleep(100);
            r.translate(-20, 0.1);
            sleep(100);
            r.rotate("ccw", 0.1, 23);
            sleep(100);
            r.translate(-23.3,0.15);
        } else if (samplePos == "center") {
            r.strafe(4, 0.4);
            r.translate(-40, 0.1);
        } else {
            r.strafe(16, 0.1);
            sleep(100);
            r.translate(-20, 0.1);
            sleep(100);
            r.rotate("cw", 0.1, 23);
            sleep(100);
            r.translate(-23.3,0.15);
        }
        sleep(100);

//            if ((detector.getScreenPosition().x < 400 && detector.getScreenPosition().x > 200) && detector.getScreenPosition().y >= 250) {
//                telemetry.addData("Position:", "Center");
//                telemetry.update();
//                //make this less before running
//
//                //TODO yesterday it was 56, i changed to 54, need to test
//                r.translate(54, -0.15);
//
//
//
//            }
//            else {
//                r.translate(4,-0.15);
//                r.rotate("ccw", 0.15, 35);
//                telemetry.addData("xpos", detector.getScreenPosition().x);
//                double currentXPos = detector.getScreenPosition().x;
//                double currentYPos = detector.getScreenPosition().y;
//                telemetry.update();
//                sleep(1000);
//                if ((currentXPos < 400 && currentXPos > 200) && currentYPos >= 250){
//                    r.translate(30.8, -0.15);
//                    r.rotate("cw",0.1,33);
//                    r.translate(33.3,-0.15);
//
//                }
//                else{
//                    r.rotate("cw", 0.1, 35);
//                    r.translate(30.8, -0.15);
//                    r.rotate("ccw",0.05,32);
//                    r.translate(33.3,-0.15);
//
//
//                }
//
//            }




            //placeholder for dropping the team marker
            sleep(100);
        r.servoMotors[0].setPosition(0);
        sleep(100);

        //rotate to 45deg instead of .5?
            r.rotate("cw",0.1,45.5);
            sleep(100);
            r.strafe(11, 0.1);
            sleep(100);
            r.strafe(-3, 0.1);
            sleep(100);
            r.translate(75,0.2);


            //.addData("xpos",detector.getScreenPosition().x);



            telemetry.update();








    }
}
