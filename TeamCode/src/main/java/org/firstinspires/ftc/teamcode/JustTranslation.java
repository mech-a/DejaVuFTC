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

/**
 * Autonomous made to test out translation and other functions related to movement.
 * Made By Gaurav
 */

@Autonomous(name="Testing Translation", group="Testing")
//@Disabled
public class JustTranslation extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, DcMotor.RunMode.RESET_ENCODERS, telemetry);

        //Translate forward at a speed of 0.6 1120 *COUNTS* while a
        //robot.translate("fwd", 0.6, 1120, a);

        //Translate forward at a speed of 0.6 12 *INCHES* while a
        //robot.translate("left", 0.4, 12.0, a);

        //Translate (rotate) clockwise using counts at a speed of 0.2 ninety degrees while a
        //DO NOT USE THIS THERE IS A BUG
        //robot.translate("cw", 0.2, robot.ninetyDegreesInInches, a);

        //Rotate clockwise using gyro 90 degrees at a speed of 0.2 while a
        //robot.rotate("cw", 0.2, 90, a);

        //Rotate counter clockwise 135 degrees at a speed of 0.3 while a
        //robot.rotate("ccw", 0.3, 135, a);
        boolean a = opModeIsActive();
        waitForStart();

        /*
        robot.translate("bk", 0.2, 1120, a);
        sleep(250);
        robot.translate("fwd", 0.2, 2 * 1120, a);
        sleep(250);
        robot.translate("cw", 0.2, robot.ninetyDegreesInInches, a);
        sleep(250);
        robot.translate("ccw", 0.2, robot.turnAroundInInches, a);
        sleep(250);
        */
        robot.rotate("cw", 0.3, 90, a);
        sleep(250);
        robot.rotate("ccw", 0.4, 135, a);
        sleep(250);


    }
}
