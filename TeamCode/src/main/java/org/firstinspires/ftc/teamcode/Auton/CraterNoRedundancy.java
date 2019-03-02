package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Enums.GoldPosition;
import org.firstinspires.ftc.teamcode.dependencies.Robot;


@Autonomous(name = "Crater Alternative", group = "Auton")
public class CraterNoRedundancy extends LinearOpMode {

    Robot r = new Robot(this, Enums.OpModeType.AUTON);

    //constants that will probably be moved to the Constants class
    private double speed = 0.16;
    private int time = 50;

    @Override public void runOpMode() {
        r.start(hardwareMap, telemetry);
        r.init();

        //CameraDevice.getInstance().setFlashTorchMode(false);


        telemetry.addData("Stat" ,"Initialized");
        telemetry.update();

        waitForStart();

        //detach from the lander

        r.armMotors[0].setPower(-1);

        sleep(200);
        r.servoMotors[1].setPosition(0.35);
        sleep(200);

        //r.positionDrive(0,840,0.3 );

        r.armMotors[0].setPower(0.5);
        while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 782) {}
        r.armMotors[0].setPower(0);

        sleep(100);



        //this is an enum
        //save the detected position of the gold mineral
        GoldPosition x = r.getGoldPosition();
        //GoldPosition x = GoldPosition.RIGHT;


        telemetry.addData("Position:", x);
        telemetry.update();

        r.translate(Enums.Direction.RIGHT, 4, speed);

//        sleep(time);
//        r.rotate("cw", speed/2, 0);
//        sleep(time);

        r.translate(Enums.Direction.BACK, 15, speed);

        if(x==GoldPosition.MIDDLE) {
            r.translate(Enums.Direction.LEFT,7, speed);
        }
        else if (x==GoldPosition.RIGHT) {
            r.translate(Enums.Direction.LEFT, 21, speed);
        }
        else {
            r.translate(Enums.Direction.RIGHT, 8, speed);
        }

        r.translate(Enums.Direction.BACK, 7, speed);
        r.translate(Enums.Direction.FWD, 7, speed);

        if(x==GoldPosition.MIDDLE) {
            r.translate(Enums.Direction.RIGHT,39+7, speed);
        }
        else if (x==GoldPosition.RIGHT) {
            r.translate(Enums.Direction.RIGHT, 21+39, speed);
        }
        else {
            r.translate(Enums.Direction.RIGHT, 39-8, speed);
        }

        r.rotate("ccw", speed, 130);
        sleep(time);
        r.translate(Enums.Direction.LEFT, 5, speed);
        sleep(time);
        r.translate(Enums.Direction.BACK, 37, speed);
        sleep(time);
        r.positionDrive(2, 600,0.75);
//        sleep(500);
//        r.positionDrive(2, -600,0.75);
        sleep(time);
        r.translate(Enums.Direction.FWD, 70, speed+0.1);


    }
}
