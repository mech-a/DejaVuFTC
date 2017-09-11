package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by gbhat on 8/20/2017.
 */

@TeleOp(name="Mecanum Drive", group="DriveOPs")
@Disabled
public class MecanumDrive extends LinearOpMode{
    HWRobot robot = new HWRobot();
     float[][] joystick = {
             {
                     gamepad1.left_stick_x, -gamepad1.left_stick_y,
                     gamepad1.right_stick_x, -gamepad1.right_stick_y
             },
             {
                     gamepad2.left_stick_x, -gamepad2.left_stick_y,
                     gamepad2.right_stick_x, -gamepad2.right_stick_y
             }
     };

     boolean[][] buttons = {
             {
                     gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y,
             },
             {
                     gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y
             }
     };

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        prompt(telemetry, "Init", "HW initialized");

        waitForStart();

        while(opModeIsActive()) {

        }
    }

    private void prompt(Telemetry telemetry, String prefix, String cmd) {
        telemetry.addData(prefix, cmd);
        telemetry.update();
    }
}
