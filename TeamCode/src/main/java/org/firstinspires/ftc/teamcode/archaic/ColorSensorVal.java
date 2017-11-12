package org.firstinspires.ftc.teamcode.archaic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.called.HWRobot;

/**
 * Autonomous made for color sensor values
 * Made By Gaurav
 */

@TeleOp(name="Color Sensor", group="Testing")
//@Disabled
public class ColorSensorVal extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();


    @Override
    public void runOpMode() {
        robot.getOpModeData(telemetry,hardwareMap);robot.init("sensors");


        waitForStart();
        runtime.reset();


        while(opModeIsActive()) {
            robot.refreshHSV();

            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("Hue", robot.hsv[0]);
            telemetry.addData("Saturation", robot.hsv[1]);
            telemetry.addData("Value", robot.hsv[2]);

        }
    }
}
