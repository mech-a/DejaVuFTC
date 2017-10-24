package org.firstinspires.ftc.teamcode.archaic;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Trial autonomous aka kalie messing around w stuff
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Autonomous OPs")
@Disabled
public class InitialAuton extends LinearOpMode {

    // Declare OpMode members: motors, servos, sensors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor mtrFL, mtrFR, mtrBL, mtrBR;
    private ColorSensor sensorColor;
    int a = 0;
    int b = 0;

    // METHODS!

    public void movement (String direction, int distance) {
        if (direction == "left") {
            a = -1;
            b = 1;
        } else if (direction == "right") {
            a = 1;
            b = -1;
        } else if (direction == "forward") {
            a = 1;
            b = 1;
        } else if (direction == "backward") {
            a = -1;
            b = -1;
        }

        mtrFL.setPower(a);
        mtrBR.setPower(a);

        mtrFR.setPower(b);
        mtrBL.setPower(b);

        distance(distance);
    }

    public void jewel (double servoPosition){
        //jewelSrv.setPosition(servoPosition);
    }

    public void distance (int distance) {
        mtrFR.setTargetPosition(distance);
        mtrBL.setTargetPosition(distance);
        mtrFL.setTargetPosition(distance);
        mtrBR.setTargetPosition(distance);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters.
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        mtrFL = hardwareMap.dcMotor.get("fl_drive");
        mtrFR = hardwareMap.dcMotor.get("fr_drive");
        mtrBL = hardwareMap.dcMotor.get("bl_drive");
        mtrBR = hardwareMap.dcMotor.get("br_drive");

        // Set all motors to run using encoders.
        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // An array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // Values is a reference to the hsvValues array. Final means it cannot be changed.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout",
                "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Wait for the game to start (driver presses PLAY).
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP).
        while (opModeIsActive()) {

            // (distance)
            movement("right", 1220);
            jewel(0.6);
            movement("left", 1220);
            movement("backward", 1220);
            if (sensorColor.red() > 127) {
                movement("forward", 1220);
            } else {
                movement("left", 1220);
            }

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }





}