package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Autonomous made to test out translation and other functions related to movement.
 * Made By Gaurav
 */

@Autonomous(name="CookieCutter Blue Back", group="Testing")
//@Disabled
public class CookieCutterAutonomous extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();
    private String pictograph;
    private int cryptoboxAdditionalCounts = 0;
    int countsPerInch = 252;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, DcMotor.RunMode.RESET_ENCODERS, telemetry);
        boolean a = opModeIsActive();

        waitForStart();

        //Finds Vumark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
        pictograph = robot.getVuMark(vuMark, a);
        sleep(100);

        //Translates into position for jewel
        robot.translate("right", 0.2, 5000, a);
        robot.srvJewel.setPosition(0.5);
        robot.refreshHSV();

        //if the ball is blue
        if(robot.hsv[0] > 215 && robot.hsv[0] < 270) {
            robot.translate("fwd", 0.2, 350, a);
            robot.srvJewel.setPosition(0);
            robot.translate("bk", 0.2, 350, a);
        }
        else {
            robot.translate("bk", 0.2, 350, a);
            robot.srvJewel.setPosition(0);
            robot.translate("fwd", 0.2, 350, a);
        }

        //TODO implement while loop checking for color of tape underneath robot to stop, then make additional counts a value * 1,2,3 for better movement

        //Decide how far to go for cryptobox
        if(pictograph.equals("RIGHT")) {
            robot.translate("fwd", 0.2, 500, a);
        }
        else if(pictograph.equals("CENTER")) {
            robot.translate("fwd", 0.2,  2 * 500, a);
        }
        else if(pictograph.equals("LEFT")) {
            robot.translate("fwd", 0.2,  3 * 500, a);
        }

        //Rotate and release jewel
        robot.rotate("cw", 0.2, 90, a);
        robot.translate("fwd", 0.1, 1120, a);
        robot.srvL.setPosition(0.75);
        robot.srvR.setPosition(0.25);


    }
}
