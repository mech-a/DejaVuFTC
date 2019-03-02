package org.firstinspires.ftc.teamcode.Auton;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.firstinspires.ftc.teamcode.DogeCVTesting.CustomGoldDetector;
import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc
        .teamcode.dependencies.Enums.GoldPosition;
import org.firstinspires.ftc.teamcode.dependencies.Robot;

import static org.firstinspires.ftc.teamcode.dependencies.Constants.SERVO_UNLOCKED;


@Autonomous(name = "Depot Side", group = "Auton")
public class DepotSideAuton extends LinearOpMode {

    Robot r = new Robot(this, Enums.OpModeType.AUTON);

    //constants that will probably be moved to the Constants class
    private double speed = 0.15;
    private int time = 50;

    @Override public void runOpMode() {
        r.start(hardwareMap, telemetry);
        r.init();

        telemetry.addData("Stat:" ,"Initialized");
        telemetry.update();

        waitForStart();



        r.positionDrive(2, 100,0.75);


        //detach from the lander

        r.armMotors[0].setPower(-1);

        sleep(200);
        r.servoMotors[1].setPosition(0.35);
        sleep(100);

        //r.positionDrive(0,840,0.3 );

          r.armMotors[0].setPower(0.5);
        while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 780) {}
        r.armMotors[0].setPower(0);


        sleep(100);

        //this is an enum
        //save the detected position of the gold mineral
        GoldPosition x = r.getGoldPosition();

        //translate out of the hook

        r.translate(Enums.Direction.RIGHT, 4, 0.1);

        r.translate(Enums.Direction.BACK, 13,-0.15);

        //TODO check if last motor hits pos or first motor hits position

        //the robot moves to knock out the detected mineral in its position
        //net backwards distance is 45 inches in every case
        telemetry.addData("Position:", x);
        telemetry.update();




        if (x==Enums.GoldPosition.MIDDLE) {
            telemetry.update();
            r.translate(Enums.Direction.LEFT,4, speed);
            r.translate(Enums.Direction.BACK,38, speed);
        }
        else if (x==Enums.GoldPosition.RIGHT) {
            sleep(4*1000);
            r.translate(Enums.Direction.LEFT, 4+15, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,30, speed);
            sleep(time);
            r.translate(Enums.Direction.RIGHT, 14, speed);
            sleep(time);
        }else{
            r.translate(Enums.Direction.RIGHT, 12, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,30, speed);
            sleep(time);
            r.translate(Enums.Direction.LEFT, 14, speed);
            sleep(time);
        }

        //Drop the team marker in the depot
        r.positionDrive(2, 600,0.75);
        sleep(100);


        //Use wall to align and translate into the opposing alliance's crater
        r.rotate("cw",0.1,32);

//        r.translate(Enums.Direction.RIGHT, 16, speed+0.1);
//        sleep(time);
//        r.translate(Enums.Direction.LEFT, 4, speed);
//        sleep(time);


        if(x== Enums.GoldPosition.RIGHT) {
            r.translate(Enums.Direction.RIGHT, 15, speed+0.1);
        }
        else if (x== Enums.GoldPosition.LEFT) {
            r.translate(Enums.Direction.RIGHT, 12, speed+0.1);
        }
        else {
            r.translate(Enums.Direction.RIGHT, 10, speed+0.1);
        }
        //IF MIDDLE GO 7INCHES towards the wall
        //
        //two inch more on left
        //75-71






        //MAKE THIS RUN LESS INCHES
        sleep(time);
        r.translate(Enums.Direction.FWD,70,0.2);

        telemetry.update();
    }
}
