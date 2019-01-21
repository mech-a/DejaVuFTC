package org.firstinspires.ftc.teamcode.preregional.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.preregional.dependencies.Enums;
import org.firstinspires.ftc.teamcode.preregional.dependencies.Enums.GoldPosition;
import org.firstinspires.ftc.teamcode.preregional.dependencies.Robot;


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

        //detach from the lander

        r.armMotors[0].setPower(-1);

        sleep(200);
        r.servoMotors[1].setPosition(0.35);
        sleep(500);

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
            r.translate(Enums.Direction.BACK,10, speed);
        }
        else if (x==Enums.GoldPosition.RIGHT) {
            r.translate(Enums.Direction.LEFT, 4+15, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,10, speed);
            sleep(time);
//            r.translate(Enums.Direction.RIGHT, 14, speed);
//            sleep(time);
        }else{
            r.translate(Enums.Direction.RIGHT, 12, speed);
            sleep(time);
            r.translate(Enums.Direction.BACK,10, speed);
            sleep(time);
//            r.translate(Enums.Direction.LEFT, 14, speed);
//            sleep(time);

        telemetry.update();
        }
    }
}
