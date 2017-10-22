package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/**
 * Autonomous made to test out translation and other functions related to movement.
 * Made By Gaurav
 */

@TeleOp(name="ServoGyro", group="Testing")
//@Disabled
public class ServoGyro extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();

    // The IMU sensor object

    // State used for updating telemetry
    Orientation angles;
    //Acceleration gravity;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, DcMotor.RunMode.RESET_ENCODERS, telemetry);
        // Set up our telemetry dashboard
        //composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // Loop and update the dashboard
        while (opModeIsActive()) {
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading" , AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.update();

        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    /*

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                //angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //gravity  = imu.getGravity();
            }
        });



        telemetry.addLine().addData("heading", new Func<double>() {
                    @Override public double value() {
                        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
                        //return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle))
                        // return formatAngle(angles.angleUnit, angles.firstAngle);   //display heading (maybe yaw?)
                    }
                });
    }


        //----------------------------------------------------------------------------------------------
        // Formatting
        //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

*/

}
