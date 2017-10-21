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
@Disabled
public class TestingTranslation extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, DcMotor.RunMode.RESET_ENCODERS, telemetry);


        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;





        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("Alpha", robot.sensorColor.alpha());
        telemetry.addData("Red  ", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue ", robot.sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);

        //Translate forward at a speed of 0.6 1120 *COUNTS* while opModeIsActive()
        robot.translate("fwd", 0.6, 1120, opModeIsActive());

        //Translate forward at a speed of 0.6 12 *INCHES* while opModeIsActive()
        robot.translate("left", 0.4, 12.0, opModeIsActive());



    }
}
