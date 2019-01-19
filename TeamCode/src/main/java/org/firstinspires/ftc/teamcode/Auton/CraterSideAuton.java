package org.firstinspires.ftc.teamcode.Auton;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DogeCVTesting.CustomGoldDetector;
import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Enums.GoldPosition;
import org.firstinspires.ftc.teamcode.dependencies.Robot;


@Autonomous(name = "Crater Side", group = "Auton")
public class CraterSideAuton extends LinearOpMode {

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

        r.armMotors[0].setPower(-0.25);

        sleep(100);
        r.servoMotors[1].setPosition(0.35);
        sleep(100);

        //r.positionDrive(0,840,0.3 );

        r.armMotors[0].setPower(0.5);
        while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 840) {}
        r.armMotors[0].setPower(0);

        sleep(100);

        //Enum goldPos = r.getGoldPosition();
        GoldPosition x = r.getGoldPosition();
        //Enum goldPos = Enums.Goldosition.MIDDLE;

        r.translate(Enums.Direction.RIGHT, 2, 0.25);

        r.translate(Enums.Direction.BACK, 4,-0.15);

        //TODO check if last motor hits pos or first motor hits position

        if (x==Enums.GoldPosition.MIDDLE) {
            telemetry.addData("Position:", "Center");
            telemetry.update();
            //make this less before running
            r.translate(Enums.Direction.LEFT, 2, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,24, speed);
            sleep(time);
            r.translate(Enums.Direction.FWD, 10, speed);
            sleep(time);
            r.rotate("ccw", speed, 80);
            sleep(time);
            r.translate(Enums.Direction.BACK, 40, speed);
            }
        else if (x==Enums.GoldPosition.RIGHT) {
            r.translate(Enums.Direction.BACK, 13,speed);
            sleep(time);
            r.translate(Enums.Direction.LEFT, 14, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,11, speed);
            sleep(time);
            r.translate(Enums.Direction.FWD, 10, speed);
            sleep(time);
            r.rotate("ccw", speed, 80);
            sleep(time);
            r.translate(Enums.Direction.BACK, 52, speed);
            sleep(time);
        }else{
            r.translate(Enums.Direction.BACK, 13,speed);
            sleep(time);
            r.translate(Enums.Direction.RIGHT, 10, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,11, speed);
            sleep(time);
            r.translate(Enums.Direction.FWD, 10, speed);
            sleep(time);
            r.rotate("ccw", speed, 80);
            sleep(time);
            r.translate(Enums.Direction.BACK, 38, speed);
            sleep(time);
        }

        r.rotate("ccw", speed, 135);
        sleep(time);
        r.translate(Enums.Direction.LEFT, 14, speed+0.1);
        sleep(time);
        r.translate(Enums.Direction.RIGHT, 2, speed);
        sleep(time);
        r.translate(Enums.Direction.BACK,32,0.2);

        //placeholder for dropping the team marker
        r.positionDrive(2, 800,0.5);
        sleep(100);
        r.positionDrive(2, 0, 0.5);

        r.translate(Enums.Direction.FWD, 75, 0.5);

        telemetry.update();
    }
}
