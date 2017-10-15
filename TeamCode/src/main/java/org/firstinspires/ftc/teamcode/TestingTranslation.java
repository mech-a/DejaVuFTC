package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Autonomous made to test out translation and other functions related to movement.
 * Made By Gaurav
 */

@Autonomous(name="Testing Translation", group="Testing")
@Disabled
public class TestingTranslation extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, DcMotor.RunMode.RESET_ENCODERS, telemetry);




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Translate forward at a speed of 0.6 1120 *COUNTS* while opModeIsActive()
        robot.translate("fwd", 0.6, 1120, opModeIsActive());

        //Translate forward at a speed of 0.6 12 *INCHES* while opModeIsActive()
        robot.translate("left", 0.4, 12.0, opModeIsActive());


    }
}
